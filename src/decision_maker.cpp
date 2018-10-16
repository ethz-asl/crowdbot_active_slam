#include <decision_maker.h>


DecisionMaker::DecisionMaker(ros::NodeHandle nh, ros::NodeHandle nh_)
    : nh_(nh), nh_private_(nh_) {
  // Get ros param
  nh_private_.param("primitive_filename", primitive_filename_, std::string(""));
  nh_private_.param("exploration_type", exploration_type_, std::string("utility"));

  // Init finished bool
  finished_ = false;

  // Publisher
  plan_pub_ = nh_.advertise<nav_msgs::Path>("plans_path", 1);

  // Subscriber
  map_sub_ = nh_.subscribe("/occupancy_map", 1,
                           &DecisionMaker::mapCallback, this);

  // Init service clients
  frontier_exploration_client_ = nh_.serviceClient
        <crowdbot_active_slam::get_frontier_list>
        ("/frontier_exploration/frontier_exploration_service");
  map_recalculation_client_ = nh_.serviceClient
        <crowdbot_active_slam::map_recalculation>
        ("/map_recalculation_service");
  get_plan_move_base_client_ = nh_.serviceClient
        <nav_msgs::GetPlan>("/move_base/make_plan");
  utility_calc_client_ = nh_.serviceClient
        <crowdbot_active_slam::utility_calc>
        ("/utility_calc_service");

  // Init SBPL env
  // set the perimeter of the robot
  std::vector<sbpl_2Dpt_t> perimeter;

  // See costmap_common_params.yaml for correct values, here for easyness left this way
  double robot_length = 0.3;
  double robot_width = 0.3;
  createFootprint(perimeter, robot_width, robot_length);

  width_ = 1000;
  height_ = 1000;
  resolution_ = 0.05;

  env_.InitializeEnv(width_, height_, 0, //map data
                                      0, 0, 0, // start
                                      0, 0, 0, // goal
                                      0, 0, 0, // goal tolerance
                                      perimeter, resolution_,
                                      0.5, // nominal vel (m/s)
                                      1.0, // time to turn 45 degs in place (s)
                                      20, // obstacle threshold
                                      primitive_filename_.c_str());

  // Planner
  bool bsearch = false;
  planner_ = new ADPlanner(&env_, bsearch);
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
                             unsigned int width, unsigned int height){
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
  // Update cost
  unsigned int ix, iy;
  for (int i = 0; i < latest_map_msg_.data.size(); i++){
    idToCell(i, ix, iy, width_, height_);
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
  // Check if map already subscribed
  if (!map_initialized_){
    ROS_WARN("Map was not subscribed until now!");
    return;
  }

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
    finished_ = true;
    ROS_INFO("Exploration finished!");
    return;
  }

  nav_msgs::Path action_plan;
  nav_msgs::GetPlan get_plan;
  crowdbot_active_slam::utility_calc utility;
  std::vector<double> utility_vec;
  std::vector<int> path_sizes;
  std::vector<double>::iterator max_utility;
  std::vector<int>::iterator path_sizes_it;
  int goal_id;

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

    if (exploration_type_ == "utility"){
      // Get utility of action plan
      utility.request.plan = action_plan;
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

  if (exploration_type_ == "utility"){
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

  crowdbot_active_slam::map_recalculation map_srv;
  if (map_recalculation_client_.call(map_srv))
  {
    ROS_INFO("Map recalculation call successfull");
  }
  else
  {
    ROS_ERROR("Failed to call service map_recalculation");
  }
}

void DecisionMaker::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr &map_msg){
  latest_map_msg_ = *map_msg;
  map_initialized_ = true;
  startExploration();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_maker");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  DecisionMaker decision_maker(nh, nh_);
  ros::spin();

  return 0;
}
