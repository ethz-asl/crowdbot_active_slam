#include <decision_maker.h>


DecisionMaker::DecisionMaker(ros::NodeHandle nh, ros::NodeHandle nh_)
    : nh_(nh), nh_private_(nh_) {
  // Get ros param
  nh_private_.param("exploration_type", exploration_type_, std::string("utility"));

  // Publisher
  plan_pub_ = nh_.advertise<nav_msgs::Path>("plans_path", 1);
  cancel_move_base_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

  // Subscriber
  joy_sub_ = nh_.subscribe("/joy", 10, &DecisionMaker::joyCallback, this);

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
  clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>
                          ("/move_base/clear_costmaps");

  nh_.getParam("/graph_optimisation/map_width", map_width_);
  nh_.getParam("/graph_optimisation/map_height", map_height_);
  nh_.getParam("/graph_optimisation/map_resolution", map_resolution_);
  nh_.getParam("/graph_optimisation/node_dist_linear", node_dist_linear_);
  nh_.getParam("/graph_optimisation/node_dist_angular", node_dist_angular_);
  nh_.getParam("/graph_optimisation/loop_closing_radius", lc_radius_);

  // Save start time
  start_time_ = ros::Time::now();
  // Wait until time is not 0
  while (start_time_.toSec() == 0) start_time_ = ros::Time::now();

  //
  return_to_start_ = false;
}

DecisionMaker::~DecisionMaker() {}

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

void DecisionMaker::idToCell(unsigned int id, unsigned int& x, unsigned int& y,
                             int width, int height){
    y = id / width;
    x = id - y * width;
  }

void DecisionMaker::startExploration(){
  // Get frontiers
  crowdbot_active_slam::get_frontier_list frontier_srv;
  ros::Duration(0.5).sleep();
  if (frontier_exploration_client_.call(frontier_srv))
  {
    ROS_INFO("Frontier exploration call successfull");
  }
  else
  {
    ROS_ERROR("Failed to call service frontier_exploration");
  }

  // Check if frontier list empty
  if (frontier_srv.response.frontier_list.size() == 0 && return_to_start_){
    ROS_INFO("Exploration finished!");
    end_time_ = ros::Time::now();

    // Save general test information
    saveGeneralResults();

    // Save the occupancy grid map
    saveGridMap();

    // Save uncertainties along path
    crowdbot_active_slam::service_call uncertainty_srv;
    uncertainty_srv.request.save_path = save_directory_path_;
    if (uncertainty_client_.call(uncertainty_srv)){
      ROS_INFO("Uncertainties of path have been saved!");
    }
    else{
      ROS_INFO("Uncertainty service failed!");
    }

    // Shutdown
    ROS_INFO("This node will be shutdown now!");
    ros::shutdown();
    return;
  }
  else if (frontier_srv.response.frontier_list.size() == 0 && !return_to_start_){
    geometry_msgs::Pose2D start_pose;
    start_pose.x = 0;
    start_pose.y = 0;
    start_pose.theta = 0;
    frontier_srv.response.frontier_list.push_back(start_pose);
    return_to_start_ = true;
  }

  nav_msgs::Path action_plan;
  nav_msgs::GetPlan get_plan;
  crowdbot_active_slam::utility_calc utility;
  std::vector<double> utility_vec;
  std::vector<double> path_sizes;
  std::vector<double>::iterator max_utility;
  std::vector<double>::iterator path_sizes_it;
  int goal_id = 0;

  for (int i = 0; i < frontier_srv.response.frontier_list.size(); i++){
    // Get action plan
    get_plan.request.start = pose2DToPoseStamped(frontier_srv.response.start_pose);
    get_plan.request.goal = pose2DToPoseStamped(frontier_srv.response.frontier_list[i]);
    // if using NavfnROS set tolerance to 0.0. This tolerance is used from move_base
    // to find a goal. But NavfnROS has itself a tolerance, which can be set
    // from rosparams. If both values set can screw things up.
    // If using Global planner use this tolerance as itself is not using rosparam
    // tolerance!!
    get_plan.request.tolerance = 0.0;
    get_plan_move_base_client_.call(get_plan);
    if (get_plan.response.plan.poses.size() == 0){
      frontier_srv.response.frontier_list.erase(frontier_srv.response.frontier_list.begin() + i);
      i -= 1;
      continue;
    }
    action_plan = get_plan.response.plan;
    action_plan.header.frame_id = "/map";
    plan_pub_.publish(action_plan);

    if (exploration_type_ == "shortest_frontier"){
      double length = 0;
      if (action_plan.poses.size() < 3){
        length = 10000;
      }
      else {
        for (int j = 1; j < action_plan.poses.size(); j++){
          length += sqrt(pow(action_plan.poses[j].pose.position.x -
                             action_plan.poses[j - 1].pose.position.x, 2) +
                         pow(action_plan.poses[j].pose.position.y -
                             action_plan.poses[j - 1].pose.position.y, 2));
        }
      }
      path_sizes.push_back(length);
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

  // Check again if frontier list empty
  if (frontier_srv.response.frontier_list.size() == 0 && return_to_start_){
    ROS_INFO("Exploration finished!");
    end_time_ = ros::Time::now();

    // Save general test information
    saveGeneralResults();

    // Save the occupancy grid map
    saveGridMap();

    // Save uncertainties along path
    crowdbot_active_slam::service_call uncertainty_srv;
    uncertainty_srv.request.save_path = save_directory_path_;
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
  else if (frontier_srv.response.frontier_list.size() == 0 && !return_to_start_) {
    geometry_msgs::Pose2D start_pose;
    start_pose.x = 0;
    start_pose.y = 0;
    start_pose.theta = 0;
    frontier_srv.response.frontier_list.push_back(start_pose);
    return_to_start_ = true;
    ROS_INFO("start pose added!2");
    if (exploration_type_ == "shortest_frontier"){
      path_sizes.push_back(10);
    }
    if (exploration_type_ == "utility_standard" ||
        exploration_type_ == "utility_normalized"){
      utility_vec.push_back(10);
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

  bool finished_before_timeout = goal_client.waitForResult(ros::Duration(180.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = goal_client.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // ros::Duration(1.5).sleep();
  crowdbot_active_slam::service_call map_srv;
  if (map_recalculation_client_.call(map_srv))
  {
    ROS_INFO("Map recalculation call successfull");
  }
  else
  {
    ROS_ERROR("Failed to call service map_recalculation");
  }

  ros::Duration(1.5).sleep();
  std_srvs::Empty clear_costmap_srv;
  if (clear_costmap_client_.call(clear_costmap_srv)){
    ROS_INFO("Clear costmap call successfull");
  }
  else {
    ROS_ERROR("Failed to call service move_base/clear_costmaps");
  }
}

void DecisionMaker::saveGridMap(){
  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string save_path = save_directory_path_ + "/occupancy_grid_map.txt";

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
  save_directory_path_ = ros::package::getPath("crowdbot_active_slam") +
                    "/test_results/" + exploration_type_ + char_time;
  boost::filesystem::path dir(save_directory_path_);
  if(boost::filesystem::create_directory(dir)){}
  else std::cout << "Failed creating directory!" << std::endl;

  std::string save_path = save_directory_path_ + "/general_results.txt";

  //
  double diff_time = end_time_.toSec() - start_time_.toSec();

  //
  std::ofstream result_file(save_path.c_str());
  if (result_file.is_open()){
    // Add information to file
    result_file << "Exploration type: " << exploration_type_ << std::endl;
    result_file << "Exploration time: " << diff_time << " s" << std::endl;
    result_file << "Map width: " << map_width_ << std::endl;
    result_file << "Map height: " << map_height_ << std::endl;
    result_file << "Map resolution: " << map_resolution_ << std::endl;
    result_file << "node_dist_linear: " << node_dist_linear_ << std::endl;
    result_file << "node_dist_angular: " << node_dist_angular_ << std::endl;
    result_file << "loop_closing_radius: " << lc_radius_ << std::endl;
    result_file << "Node number: " << "FILL IN" << std::endl;
    result_file << "World: " << "FILL IN" << std::endl;

    if (exploration_type_ == "shortest_frontier"){
      result_file << "Optimality: " << "None" << std::endl;
    }
    else {
      result_file << "Optimality: " << "D-optimality" << std::endl;
    }

    // Close file
    result_file.close();
  }
  else{
    ROS_INFO("Could not open general_results.txt!");
  }
}

void DecisionMaker::joyCallback(const sensor_msgs::Joy::ConstPtr& msg){
  if (msg->buttons[2] == 1){
    actionlib_msgs::GoalID cancel;
    cancel_move_base_pub_.publish(cancel);
    ros::shutdown();
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
