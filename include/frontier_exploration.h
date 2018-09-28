#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <crowdbot_active_slam/get_frontier_list.h>


class FrontierExploration {
public:
  /**
   *  Class Constructor
   */
  FrontierExploration(ros::NodeHandle nh, ros::NodeHandle nh_);

  /**
   *  Class Destructor
   */
  ~FrontierExploration();

  /**
   *  ...
   */
  bool serviceCallback(
                  crowdbot_active_slam::get_frontier_list::Request &request,
                  crowdbot_active_slam::get_frontier_list::Response &response);

  /**
   *  ...
   */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Service, Subscriber, Publisher
  ros::ServiceServer service_;
  ros::Subscriber map_sub_;

  // Ros msgs and srv
  nav_msgs::OccupancyGrid latest_map_msg_;

  // others
  bool map_initialized_;
};

#endif // FRONTIER_EXPLORATION_H
