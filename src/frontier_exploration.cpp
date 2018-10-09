#include <frontier_exploration.h>

/**
 *  A helper function which creates map index from position information.
 */
std::vector<int> positionToMapIndex(double x, double y,
   unsigned int width, unsigned int height, float resolution){
  std::vector<int> index(2);
  index[0] = floor(x / resolution) + width / 2;
  index[1] = floor(y / resolution) + height / 2;

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

  // Init map subscriber
  map_initialized_ = false;
  map_sub_ = nh_.subscribe("/occupancy_map", 1,
                           &FrontierExploration::mapCallback, this);

  // Init publisher
  frontier_cell_pub_ = nh_.advertise<nav_msgs::GridCells>("frontier_points_grid_cell", 1);
}

FrontierExploration::~FrontierExploration() {}

bool FrontierExploration::serviceCallback(
    crowdbot_active_slam::get_frontier_list::Request &request,
    crowdbot_active_slam::get_frontier_list::Response &response) {
  // Frontier exploration calculations
  // based on implementation http://wiki.ros.org/frontier_exploration

  // Check if map already subscribed
  if (!map_initialized_){
    ROS_WARN("Service aborted! Map was not subscribed until now!");
    return false;
  }

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

  // Save map information
  unsigned int width = latest_map_msg_.info.width;
  unsigned int height = latest_map_msg_.info.height;
  float resolution = latest_map_msg_.info.resolution;

  // Update map with freespace threshold
  for (int i = 0; i < latest_map_msg_.data.size(); i++){
    if (int(latest_map_msg_.data[i]) <= 25 && int(latest_map_msg_.data[i]) != -1){
      latest_map_msg_.data[i] = 0;
    }
  }

  // Get robot position in map indices
  int robot_x_cell = positionToMapIndex(robot_x, robot_y, width, height, resolution)[0];
  int robot_y_cell = positionToMapIndex(robot_x, robot_y, width, height, resolution)[1];

  // Print result
  std::cout << "Robot cell: (" << robot_x_cell << ", " << robot_y_cell << ")" << std::endl;

  // Init frontier and visited flag to keep record while searching for frontiers
  std::vector<bool> frontier_flag(width * height, false);
  std::vector<bool> visited_flag(width * height, false);

  // Init breadth first search queue object
  std::queue<unsigned int> breadth_first_search;

  // Init with robot pose as a starting point of the search.
  // Assumption: robot pose is a free space
  breadth_first_search.push(robot_x_cell + robot_y_cell * width);
  visited_flag[breadth_first_search.front()] = true;

  // Search for frontiers
  while (!breadth_first_search.empty()){
    unsigned int id = breadth_first_search.front();
    breadth_first_search.pop();

    std::vector<unsigned int> neighbour_vec = neighbour4(id, width, height);
    for (int i = 0; i < neighbour_vec.size(); i++){
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
        geometry_msgs::Pose2D frontier_centroid = getFrontierCentroid(
          neighbour_vec[i], frontier_flag, width, height, resolution);

          // Check if values are zero -> if yes frontier size was too small
        if (frontier_centroid.x != 0 && frontier_centroid.y != 0)
          response.frontier_list.push_back(frontier_centroid);
      }
    }
  }
  // Generate GridCells msg of frontier centroids
  nav_msgs::GridCells frontier_points_msg;
  frontier_points_msg.header.frame_id = "map";
  frontier_points_msg.cell_width = 0.1;
  frontier_points_msg.cell_height = 0.1;
  for (int i = 0; i < response.frontier_list.size(); i++){
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

geometry_msgs::Pose2D FrontierExploration::getFrontierCentroid(
                  unsigned int initial_cell, std::vector<bool>& frontier_flag,
                  unsigned int width, unsigned int height, float resolution){

  geometry_msgs::Pose2D centroid_pose2D;
  unsigned int size = 1;
  double centroid_x, centroid_y;

  // Get x,y world coordinates of initial cell id and init centroid values
  double ix, iy;
  idToWorldXY(initial_cell, ix, iy, width, height, resolution);
  centroid_x = ix;
  centroid_y = iy;

  // Init breadth first search queue
  std::queue<unsigned int> breadth_first_search;
  breadth_first_search.push(initial_cell);

  // Search for frontiers
  while (!breadth_first_search.empty()){
    unsigned int id = breadth_first_search.front();
    breadth_first_search.pop();

    std::vector<unsigned int> neighbour8_vec = neighbour8(id, width, height);
    for (int i = 0; i < neighbour8_vec.size(); i++){
      // Check if cell is unknown and has not been marked as a frontier cell
      if (latest_map_msg_.data[neighbour8_vec[i]] == -1 &&
          !frontier_flag[neighbour8_vec[i]]){
        bool has_free_neighbour4 = false;
        std::vector<unsigned int> neighbour4_vec =
                                    neighbour4(neighbour8_vec[i], width, height);
        // Check if a neighbour is a free cell
        for (int j = 0; j < neighbour4_vec.size(); j++){
          if (latest_map_msg_.data[neighbour4_vec[j]] == 0){
            has_free_neighbour4 = true;
          }
        }
        // if a neighbour is a free cell, mark as frontier and update centroid
        if (has_free_neighbour4){
          frontier_flag[neighbour8_vec[i]] = true;

          double new_x, new_y;
          idToWorldXY(neighbour8_vec[i], new_x, new_y, width, height, resolution);
          centroid_x += new_x;
          centroid_y += new_y;
          size++;

          breadth_first_search.push(neighbour8_vec[i]);
        }
      }
    }
  }
  // Check if size of frontier is big enough
  if (size > frontier_size_){
    centroid_x /= size;
    centroid_y /= size;
    centroid_pose2D.x = centroid_x;
    centroid_pose2D.y = centroid_y;
    centroid_pose2D.theta = 0.0;
  }
  else {
    centroid_pose2D.x = 0.0;
    centroid_pose2D.y = 0.0;
    centroid_pose2D.theta = 0.0;
  }
  return centroid_pose2D;
}

void FrontierExploration::idToWorldXY(unsigned int id, double& x, double& y,
  unsigned int width, unsigned int height, float resolution){
    unsigned int iy_cell = id / width;
    unsigned int ix_cell = id - iy_cell * width;
    x = int((ix_cell - width / 2)) * resolution + resolution / 2.0;
    y = int((iy_cell - height / 2)) * resolution + resolution / 2.0;
  }

std::vector<unsigned int> FrontierExploration::neighbour8(unsigned int id,
                                      unsigned int width, unsigned int height){
  std::vector<unsigned int> neighbour8_vec;

  if (id >= width){
    neighbour8_vec.push_back(id - width);
    if (id % width > 0){
      neighbour8_vec.push_back(id - 1);
      neighbour8_vec.push_back(id - width - 1);
    }
    if (id % width < width - 1){
      neighbour8_vec.push_back(id + 1);
      neighbour8_vec.push_back(id - width + 1);
    }
  }
  if (id < width * (height - 1)){
    neighbour8_vec.push_back(id + width);
    if (id % width > 0){
      neighbour8_vec.push_back(id + width - 1);
    }
    if (id % width < width - 1){
      neighbour8_vec.push_back(id + width + 1);
    }
  }

  return neighbour8_vec;
}

std::vector<unsigned int> FrontierExploration::neighbour4(unsigned int id,
                                      unsigned int width, unsigned int height){
  std::vector<unsigned int> neighbour4_vec;

  if (id >= width){
    neighbour4_vec.push_back(id - width);
  }
  if (id < width * (height - 1)){
    neighbour4_vec.push_back(id + width);
  }
  if (id % width > 0){
    neighbour4_vec.push_back(id - 1);
  }
  if (id % width < width - 1){
    neighbour4_vec.push_back(id + 1);
  }
  return neighbour4_vec;
}

void FrontierExploration::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr &map_msg){
  latest_map_msg_ = *map_msg;
  map_initialized_ = true;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "frontier_exploration");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  FrontierExploration frontier_exploration(nh, nh_);
  ros::spin();
  return 0;
}
