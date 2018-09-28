#include <frontier_exploration.h>

FrontierExploration::FrontierExploration(ros::NodeHandle nh,
                                         ros::NodeHandle nh_)
    : nh_(nh), nh_private_(nh_) {
  // Init service
  service_ = nh_.advertiseService("frontier_exploration_service",
                                  &FrontierExploration::serviceCallback, this);
  ROS_INFO("frontier_exploration_service started!");

  // Init subscriber
  map_initialized_ = false;
  map_sub_ = nh_.subscribe("occupancy_map", 1,
                           &FrontierExploration::mapCallback, this);
}

FrontierExploration::~FrontierExploration() {}

bool FrontierExploration::serviceCallback(
    crowdbot_active_slam::get_frontier_list::Request &request,
    crowdbot_active_slam::get_frontier_list::Response &response) {
  // Frontier exploration calculations
  geometry_msgs::Pose2D test;
  test.x = 1.0;
  test.y = 1.0;
  test.theta = 1.0;
  response.frontier_list.push_back(test);
  response.frontier_list.push_back(test);
  response.frontier_list.push_back(test);

  return true;
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
