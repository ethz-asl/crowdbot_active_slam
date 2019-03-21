#include <frontier_exploration.h>

/**
 *  A helper function which creates map index from position information.
 */
std::vector<int> positionToMapIndex(double x, double y,
  int map_width, int map_height, float map_resolution){
  std::vector<int> index(2);
  index[0] = floor(x / map_resolution) + map_width / 2;
  index[1] = floor(y / map_resolution) + map_height / 2;

  return index;
}

FrontierExploration::FrontierExploration(ros::NodeHandle nh,
                                         ros::NodeHandle nh_)
    : nh_(nh), nh_private_(nh_) {
  // Get ros param
  nh_private_.param<int>("frontier_size", frontier_size_, 50);

  // Init service
  service_ = nh_.advertiseService("frontier_exploration_service",
                                  &FrontierExploration::serviceCallback, this);
  ROS_INFO("frontier_exploration_service started!");

  // Init service clients
  get_map_client_ = nh_.serviceClient<crowdbot_active_slam::get_map>
                    ("/get_map_service");

  // Init publisher
  frontier_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("frontier_points_grid_cell", 1);

  nh_.getParam("/graph_optimisation/map_width", map_width_);
  nh_.getParam("/graph_optimisation/map_height", map_height_);
  nh_.getParam("/graph_optimisation/map_resolution", map_resolution_);
}

FrontierExploration::~FrontierExploration() {}

bool FrontierExploration::serviceCallback(
    crowdbot_active_slam::get_frontier_list::Request &request,
    crowdbot_active_slam::get_frontier_list::Response &response) {
  // Frontier exploration calculations
  // based on implementation http://wiki.ros.org/frontier_exploration

  // Init Transform listener and get current robot pose
  tf::StampedTransform robot_pose_tf;
  robot_pose_listener_.waitForTransform("map", "base_link", ros::Time(0),
                                ros::Duration(1.0));
  robot_pose_listener_.lookupTransform("map", "base_link",
                                  ros::Time(0), robot_pose_tf);

  double robot_x = robot_pose_tf.getOrigin().getX();
  double robot_y = robot_pose_tf.getOrigin().getY();
  double robot_theta = tf::getYaw(robot_pose_tf.getRotation());

  response.start_pose.x = robot_x;
  response.start_pose.y = robot_y;
  response.start_pose.theta = robot_theta;

  // Service call for latest occupancy grid map
  crowdbot_active_slam::get_map get_map_srv;

  if (get_map_client_.call(get_map_srv))
  {
    ROS_INFO("OccupancyGrid map call successfull");
  }
  else
  {
    ROS_ERROR("Failed to call OccupancyGrid map");
  }

  latest_map_msg_ = get_map_srv.response.map_msg;

  // Update map with freespace threshold
  for (unsigned int i = 0; i < latest_map_msg_.data.size(); i++){
    if (int(latest_map_msg_.data[i]) <= 25 && int(latest_map_msg_.data[i]) != -1){
      latest_map_msg_.data[i] = 0;
    }
  }

  // Get robot position in map indices
  int robot_x_cell = positionToMapIndex(robot_x, robot_y, map_width_, map_height_, map_resolution_)[0];
  int robot_y_cell = positionToMapIndex(robot_x, robot_y, map_width_, map_height_, map_resolution_)[1];

  // Print result
  std::cout << "Robot cell: (" << robot_x_cell << ", " << robot_y_cell << ")" << std::endl;

  // Init frontier and visited flag to keep record while searching for frontiers
  std::vector<bool> frontier_flag(map_width_ * map_height_, false);
  std::vector<bool> visited_flag(map_width_ * map_height_, false);

  // Init breadth first search queue object
  std::queue<unsigned int> breadth_first_search;

  // Init with robot pose as a starting point of the search.
  // Assumption: robot pose is a free space
  if (latest_map_msg_.data[robot_x_cell + robot_y_cell * map_width_] == -1){
    if (latest_map_msg_.data[robot_x_cell + 10 + robot_y_cell * map_width_] == 0){
      breadth_first_search.push(robot_x_cell + 10 + robot_y_cell * map_width_);
    }
    else if (latest_map_msg_.data[robot_x_cell + (robot_y_cell + 10) * map_width_] == 0) {
      breadth_first_search.push(robot_x_cell + (robot_y_cell + 10) * map_width_);
    }
    else {
      ROS_WARN("Frontier exploration starts at unknown space and has no free space in front! Change orientation!");
    }
  }
  else {
    breadth_first_search.push(robot_x_cell + robot_y_cell * map_width_);
  }
  visited_flag[breadth_first_search.front()] = true;

  // Search for frontiers
  while (!breadth_first_search.empty()){
    unsigned int id = breadth_first_search.front();
    breadth_first_search.pop();

    std::vector<unsigned int> neighbour_vec = neighbour4(id, map_width_, map_height_);
    for (unsigned int i = 0; i < neighbour_vec.size(); i++){
      // Check if cell is free and has not been visited, then add to queue
      if (int(latest_map_msg_.data[neighbour_vec[i]]) == 0 &&
          !visited_flag[neighbour_vec[i]]){
        visited_flag[neighbour_vec[i]] = true;
        breadth_first_search.push(neighbour_vec[i]);
      }
      // Check if cell is unknown and has not been marked as frontier
      else if (!frontier_flag[neighbour_vec[i]] &&
               int(latest_map_msg_.data[neighbour_vec[i]]) == -1){
        frontier_flag[neighbour_vec[i]] = true;

        // Get frontier centroid around current cell
        geometry_msgs::Pose2D frontier_centroid;
        if (getFrontierCentroid(neighbour_vec[i], frontier_flag, map_width_, map_height_,
                                map_resolution_, frontier_centroid)){
          // Check if frontier is not on robot position(problem on initialisation)
          if (abs(frontier_centroid.x - robot_x) < 0.8 &&
              abs(frontier_centroid.y - robot_y) < 0.8){}
          else {
            response.frontier_list.push_back(frontier_centroid);
          }
        }
      }
    }
  }

  // Add starting pose as frontier TODO

  // Generate GridCells msg of frontier centroids
  nav_msgs::GridCells frontier_points_msg;
  frontier_points_msg.header.frame_id = "map";
  frontier_points_msg.cell_width = 0.1;
  frontier_points_msg.cell_height = 0.1;
  for (unsigned int i = 0; i < response.frontier_list.size(); i++){
    geometry_msgs::Point point;
    point.x = response.frontier_list[i].x;
    point.y = response.frontier_list[i].y;
    point.z = 0;
    frontier_points_msg.cells.push_back(point);
  }

  // Publish GridCells msg of frontier centroids
  frontier_cell_pub_.publish(frontier_points_msg);
  return true;
}

bool FrontierExploration::getFrontierCentroid(unsigned int initial_cell,
                                              std::vector<bool>& frontier_flag,
                                              int map_width,
                                              int map_height,
                                              float map_resolution,
                                              geometry_msgs::Pose2D& centroid){

  geometry_msgs::Pose2D centroid_pose2D;
  int size = 1;
  double centroid_x, centroid_y;

  // Get x,y world coordinates of initial cell id and init centroid values
  double ix, iy;
  idToWorldXY(initial_cell, ix, iy, map_width, map_height, map_resolution);
  centroid_x = ix;
  centroid_y = iy;

  // Init breadth first search queue
  std::queue<unsigned int> breadth_first_search;
  breadth_first_search.push(initial_cell);

  // Search for frontiers
  while (!breadth_first_search.empty()){
    unsigned int id = breadth_first_search.front();
    breadth_first_search.pop();

    std::vector<unsigned int> neighbour8_vec = neighbour8(id, map_width, map_height);
    for (unsigned int i = 0; i < neighbour8_vec.size(); i++){
      // Check if cell is unknown and has not been marked as a frontier cell
      if (int(latest_map_msg_.data[neighbour8_vec[i]]) == -1 &&
          !frontier_flag[neighbour8_vec[i]]){
        bool has_free_neighbour4 = false;
        std::vector<unsigned int> neighbour4_vec =
                                    neighbour4(neighbour8_vec[i], map_width, map_height);
        // Check if a neighbour is a free cell
        for (unsigned int j = 0; j < neighbour4_vec.size(); j++){
          if (int(latest_map_msg_.data[neighbour4_vec[j]]) == 0){
            has_free_neighbour4 = true;
          }
        }
        // if a neighbour is a free cell, mark as frontier and update centroid
        if (has_free_neighbour4){
          frontier_flag[neighbour8_vec[i]] = true;

          double new_x, new_y;
          idToWorldXY(neighbour8_vec[i], new_x, new_y, map_width, map_height, map_resolution);
          centroid_x += new_x;
          centroid_y += new_y;
          size++;

          breadth_first_search.push(neighbour8_vec[i]);
        }
      }
    }
  }

  // Calculate centroid pose
  centroid_x /= size;
  centroid_y /= size;
  centroid.x = centroid_x;
  centroid.y = centroid_y;
  centroid.theta = 0.0;

  // Check if size of frontier is big enough and if a neighbour is a wall
  if (size > frontier_size_){
    std::vector<int> cell = positionToMapIndex(centroid_x, centroid_y, map_width,
                                               map_height, map_resolution);
    unsigned int id = cell[0] + cell[1] * map_width;
    std::vector<unsigned int> neighbour_x_cells = neighbourXCells(id, map_width, map_height, 3);
    for (unsigned int i = 0; i < neighbour_x_cells.size(); i++){
      if (int(latest_map_msg_.data[neighbour_x_cells[i]]) > 90){
        return false;
      }
    }
    return true;
  }
  else {
    return false;
  }
}

void FrontierExploration::idToWorldXY(unsigned int id, double& x, double& y,
  int map_width, int map_height, float map_resolution){
    unsigned int iy_cell = id / map_width;
    unsigned int ix_cell = id - iy_cell * map_width;
    x = int((ix_cell - map_width / 2)) * map_resolution + map_resolution / 2.0;
    y = int((iy_cell - map_height / 2)) * map_resolution + map_resolution / 2.0;
  }

std::vector<unsigned int> FrontierExploration::neighbourXCells(unsigned int id,
                int map_width, int map_height, unsigned int n_cells){
  std::vector<unsigned int> neighbour_vec;

  int br_corner = id - n_cells - n_cells * map_width;
  for (unsigned int i = 0; i <= 2 * n_cells; i++){
    for (unsigned int j = 0; j <= 2 * n_cells; j++){
      int new_id = br_corner + j + i * map_width;
      if (new_id >= 0 && new_id <= map_width * map_height && new_id != int(id)){
        neighbour_vec.push_back(new_id);
      }
    }
  }

  return neighbour_vec;
}

std::vector<unsigned int> FrontierExploration::neighbour8(unsigned int id,
                                      int map_width, int map_height){
  std::vector<unsigned int> neighbour8_vec;
  int id_int = int(id);

  if (id_int >= map_width){
    neighbour8_vec.push_back(id_int - map_width);
    if (id_int % map_width > 0){
      neighbour8_vec.push_back(id_int - 1);
      neighbour8_vec.push_back(id_int - map_width - 1);
    }
    if (id_int % map_width < map_width - 1){
      neighbour8_vec.push_back(id_int + 1);
      neighbour8_vec.push_back(id_int - map_width + 1);
    }
  }
  if (id_int < map_width * (map_height - 1)){
    neighbour8_vec.push_back(id_int + map_width);
    if (id_int % map_width > 0){
      neighbour8_vec.push_back(id_int + map_width - 1);
    }
    if (id_int % map_width < map_width - 1){
      neighbour8_vec.push_back(id_int + map_width + 1);
    }
  }

  return neighbour8_vec;
}

std::vector<unsigned int> FrontierExploration::neighbour4(unsigned int id,
                                      int map_width, int map_height){
  std::vector<unsigned int> neighbour4_vec;
  int id_int = int(id);

  if (id_int >= map_width){
    neighbour4_vec.push_back(id_int - map_width);
  }
  if (id_int < map_width * (map_height - 1)){
    neighbour4_vec.push_back(id_int + map_width);
  }
  if (id_int % map_width > 0){
    neighbour4_vec.push_back(id_int - 1);
  }
  if (id_int % map_width < map_width - 1){
    neighbour4_vec.push_back(id_int + 1);
  }
  return neighbour4_vec;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "frontier_exploration");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  FrontierExploration frontier_exploration(nh, nh_);
  ros::spin();
  return 0;
}
