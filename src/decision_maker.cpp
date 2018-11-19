#include <decision_maker.h>


DecisionMaker::DecisionMaker(ros::NodeHandle nh, ros::NodeHandle nh_)
    : nh_(nh), nh_private_(nh_) {
  // Get ros param
  nh_private_.param("primitive_filename", primitive_filename_, std::string(""));
  nh_private_.param("exploration_type", exploration_type_, std::string("utility"));

  // Publisher
  plan_pub_ = nh_.advertise<nav_msgs::Path>("plans_path", 1);

  // Init service clients
  frontier_exploration_client_ = nh_.serviceClient
        <crowdbot_active_slam::get_frontier_list>
        ("/frontier_exploration/frontier_exploration_service");
  uncertainty_client_ = nh_.serviceClient
        <crowdbot_active_slam::service_call>
        ("/save_uncertainty_service");
  map_recalculation_client_ = nh_.serviceClient
        <crowdbot_active_slam::service_call>
        ("/map_recalculation_service");
  get_map_client_ = nh_.serviceClient<crowdbot_active_slam::get_map>
                    ("/get_map_service");
  get_plan_move_base_client_ = nh_.serviceClient
        <nav_msgs::GetPlan>("/move_base/make_plan");
  utility_calc_client_ = nh_.serviceClient
        <crowdbot_active_slam::utility_calc>
        ("/utility_calc_service");

  nh_.getParam("/graph_optimisation/map_width", map_width_);
  nh_.getParam("/graph_optimisation/map_height", map_height_);
  nh_.getParam("/graph_optimisation/map_resolution", map_resolution_);
  nh_.getParam("/graph_optimisation/node_dist_linear", node_dist_linear_);
  nh_.getParam("/graph_optimisation/node_dist_angular", node_dist_angular_);
  nh_.getParam("/graph_optimisation/loop_closing_radius", lc_radius_);

  // Init SBPL env
  // set the perimeter of the robot
  std::vector<sbpl_2Dpt_t> perimeter;

  // See costmap_common_params.yaml for correct values, here for easyness left this way
  double robot_length = 0.3;
  double robot_width = 0.3;
  createFootprint(perimeter, robot_width, robot_length);

  env_.InitializeEnv(map_width_, map_height_, 0, //map data
                                      0, 0, 0, // start
                                      0, 0, 0, // goal
                                      0, 0, 0, // goal tolerance
                                      perimeter, map_resolution_,
                                      0.5, // nominal vel (m/s)
                                      1.0, // time to turn 45 degs in place (s)
                                      20, // obstacle threshold
                                      primitive_filename_.c_str());

  // Planner
  bool bsearch = false;
  planner_ = new ADPlanner(&env_, bsearch);

  // Save start time
  start_time_ = ros::Time::now();
}

DecisionMaker::~DecisionMaker() {
  delete planner_;
}

geometry_msgs::PoseStamped pose2DToPoseStamped(geometry_msgs::Pose2D pose2D){
  geometry_msgs::Point pose_position;
  pose_position.x = pose2D.x;
  pose_position.y = pose2D.y;

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pose2D.theta);
  geometry_msgs::Quaternion pose_orientation;
  quaternionTFToMsg(q, pose_orientation);

  geometry_msgs::Pose pose;
  pose.position = pose_position;
  pose.orientation = pose_orientation;

  geometry_msgs::PoseStamped posestamped;
  posestamped.pose = pose;
  posestamped.header.frame_id = "/map";
  return posestamped;
}

unsigned int DecisionMaker::mapToSBPLCost(int occupancy){
  if (occupancy < 10 && occupancy != -1){
    return 0;
  }
  else if (occupancy == -1){
    return 0;
  }
  else{
    return 20;
  }
}

void DecisionMaker::idToCell(unsigned int id, unsigned int& x, unsigned int& y,
                             int width, int height){
    y = id / width;
    x = id - y * width;
  }

void DecisionMaker::createFootprint(std::vector<sbpl_2Dpt_t>& perimeter,
                                    double halfwidth, double halflength){
  // Implementation taken from SBPL tutorial example
  sbpl_2Dpt_t pt_m;
  pt_m.x = -halflength;
  pt_m.y = -halfwidth;
  perimeter.push_back(pt_m);
  pt_m.x = halflength;
  pt_m.y = -halfwidth;
  perimeter.push_back(pt_m);
  pt_m.x = halflength;
  pt_m.y = halfwidth;
  perimeter.push_back(pt_m);
  pt_m.x = -halflength;
  pt_m.y = halfwidth;
  perimeter.push_back(pt_m);
}

nav_msgs::Path DecisionMaker::planPathSBPL(geometry_msgs::Pose2D start_pose,
                                       geometry_msgs::Pose2D goal_pose){
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

  // Update cost
  unsigned int ix, iy;
  for (unsigned int i = 0; i < latest_map_msg_.data.size(); i++){
    idToCell(i, ix, iy, map_width_, map_height_);
    env_.UpdateCost(ix, iy, mapToSBPLCost(latest_map_msg_.data[i]));
  }

  int start_id, goal_id;

  double x_origin = latest_map_msg_.info.origin.position.x;
  double y_origin = latest_map_msg_.info.origin.position.y;

  ROS_INFO("start point (%g,%g), goal point (%g,%g) ", start_pose.x - x_origin,
   start_pose.y - y_origin, goal_pose.x - x_origin, goal_pose.y - y_origin);

  start_id = env_.SetStart(start_pose.x - x_origin, start_pose.y - y_origin, start_pose.theta);
  goal_id = env_.SetGoal(goal_pose.x - x_origin, goal_pose.y - y_origin, goal_pose.theta);

  // Planner
  planner_->set_start(start_id);
  planner_->set_goal(goal_id);
  planner_->set_initialsolution_eps(3.0);
  planner_->set_search_mode(false);

  // plan
  std::vector<int> solution_state_ids;
  double allocated_time_secs = 10.0;
  int res = planner_->replan(allocated_time_secs, &solution_state_ids);

  if (res){
    ROS_INFO("planPath: Solution found!");
  }
  else{
    ROS_INFO("planPath: Solution does not exist!");
  }

  std::vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  env_.ConvertStateIDPathintoXYThetaPath(&solution_state_ids, &sbpl_path);

  // Create a path msg of the graph node estimates
  nav_msgs::Path plan;
  plan_path_ = plan;
  plan_path_.header.frame_id = "/map";

  for(unsigned int i=0; i<sbpl_path.size(); i++){
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = sbpl_path[i].x + x_origin;
    pose.pose.position.y = sbpl_path[i].y + y_origin;
    pose.pose.position.z = 0;

    tf::Quaternion temp;
    temp.setRPY(0,0,sbpl_path[i].theta);
    quaternionTFToMsg(temp, pose.pose.orientation);

    plan_path_.poses.push_back(pose);
  }
  plan_pub_.publish(plan_path_);
  return plan_path_;
}

void DecisionMaker::startExploration(){
  // Get frontiers
  crowdbot_active_slam::get_frontier_list frontier_srv;

  if (frontier_exploration_client_.call(frontier_srv))
  {
    ROS_INFO("Frontier exploration call successfull");
  }
  else
  {
    ROS_ERROR("Failed to call service frontier_exploration");
  }

  int frontier_size = frontier_srv.response.frontier_list.size();
  if (frontier_size == 0){
    ROS_INFO("Exploration finished!");
    end_time_ = ros::Time::now();

    // Save general test information
    saveGeneralResults();

    // Save the occupancy grid map
    saveGridMap();

    // Save uncertainties along path
    crowdbot_active_slam::service_call uncertainty_srv;
    if (uncertainty_client_.call(uncertainty_srv)){
      ROS_INFO("Uncertainties of path have been saved!");
    }
    else{
      ROS_INFO("Uncertainty service failed!");
    }

    // Shutdown
    ROS_INFO("This node will be shutdown now!");
    ros::shutdown();
  }

  nav_msgs::Path action_plan;
  nav_msgs::GetPlan get_plan;
  crowdbot_active_slam::utility_calc utility;
  std::vector<double> utility_vec;
  std::vector<int> path_sizes;
  std::vector<double>::iterator max_utility;
  std::vector<int>::iterator path_sizes_it;
  int goal_id = 0;

  for (int i = 0; i < frontier_size; i++){
    // Get action plan
    get_plan.request.start = pose2DToPoseStamped(frontier_srv.response.start_pose);
    get_plan.request.goal = pose2DToPoseStamped(frontier_srv.response.frontier_list[i]);
    get_plan.request.tolerance = 0.3;
    get_plan_move_base_client_.call(get_plan);
    action_plan = get_plan.response.plan;
    action_plan.header.frame_id = "/map";
    plan_pub_.publish(action_plan);

    if (exploration_type_ == "shortest_frontier"){
      path_sizes.push_back(action_plan.poses.size());
    }

    if (exploration_type_ == "utility_standard" ||
        exploration_type_ == "utility_normalized"){
      // Get utility of action plan
      utility.request.plan = action_plan;
      utility.request.exploration_type = exploration_type_;
      utility_calc_client_.call(utility);

      std::cout << "utility: " << utility.response.utility << std::endl;
      // Save utility values in vec
      utility_vec.push_back(utility.response.utility);
    }
  }

  if (exploration_type_ == "shortest_frontier"){
    path_sizes_it = std::min_element(path_sizes.begin(), path_sizes.end());
    goal_id = std::distance(path_sizes.begin(), path_sizes_it);
  }

  if (exploration_type_ == "utility_standard" ||
      exploration_type_ == "utility_normalized"){
    // Get id of max utility
    max_utility = std::max_element(utility_vec.begin(), utility_vec.end());
    int max_utility_id = std::distance(utility_vec.begin(), max_utility);
    std::cout << "Choose this: " << utility_vec[max_utility_id] << std::endl;
    goal_id = max_utility_id;
  }

  // Action client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> goal_client("move_base", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  goal_client.waitForServer();

  ROS_INFO("Action server started, sending goal.");

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.position.x = frontier_srv.response.frontier_list[goal_id].x;
  goal.target_pose.pose.position.y = frontier_srv.response.frontier_list[goal_id].y;
  goal.target_pose.pose.position.z = 0;

  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;

  goal_client.sendGoal(goal);

  bool finished_before_timeout = goal_client.waitForResult(ros::Duration(150.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = goal_client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  ros::Duration(1).sleep();
  crowdbot_active_slam::service_call map_srv;
  if (map_recalculation_client_.call(map_srv))
  {
    ROS_INFO("Map recalculation call successfull");
  }
  else
  {
    ROS_ERROR("Failed to call service map_recalculation");
  }
}

void DecisionMaker::saveGridMap(){
  // Save current time
  std::time_t now = std::time(0);
  char char_time[100];
  std::strftime(char_time, sizeof(char_time), "_%Y_%m_%d_%H_%M_%S", std::localtime(&now));

  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string save_path = package_path + "/test_results/occupancy_grid_map" +
                          char_time + ".txt";

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

  // Save map
  std::ofstream map_file(save_path.c_str());
  if (map_file.is_open()){
    for (int i = 0; i < map_width_ * map_height_; i++){
      map_file << int(latest_map_msg_.data[i]) << std::endl;
    }
    map_file.close();
  }
  else{
    ROS_INFO("Could not open occupancy_grid_map.txt!");
  }
}

void DecisionMaker::saveGeneralResults(){
  // Save current time
  std::time_t now = std::time(0);
  char char_time[100];
  std::strftime(char_time, sizeof(char_time), "_%Y_%m_%d_%H_%M_%S", std::localtime(&now));

  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string save_path = package_path + "/test_results/general_results" +
                          char_time + ".txt";

  //
  double diff_time = end_time_.toSec() - start_time_.toSec();

  //
  std::ofstream result_file(save_path.c_str());
  if (result_file.is_open()){
    // Add information to file
    result_file << "Exploration type: " << exploration_type_ << std::endl;
    result_file << "Exploration time: " << diff_time << "s" << std::endl;
    result_file << "Map width: " << map_width_ << std::endl;
    result_file << "Map height: " << map_height_ << std::endl;
    result_file << "Map resolution: " << map_resolution_ << std::endl;
    result_file << "node_dist_linear: " << node_dist_linear_ << std::endl;
    result_file << "node_dist_angular: " << node_dist_angular_ << std::endl;
    result_file << "loop_closing_radius: " << lc_radius_ << std::endl;
    result_file << "Node number: " << "FILL IN" << std::endl;
    result_file << "World: " << "FILL IN" << std::endl;
    result_file << "Optimality: " << "FILL IN" << std::endl;

    // Close file
    result_file.close();
  }
  else{
    ROS_INFO("Could not open general_results.txt!");
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_maker");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  DecisionMaker decision_maker(nh, nh_);
  while (ros::ok()){
    decision_maker.startExploration();
  }

  return 0;
}
