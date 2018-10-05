#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <sbpl/headers.h>

#include <crowdbot_active_slam/map_recalculation.h>
#include <crowdbot_active_slam/get_frontier_list.h>
#include <move_base_msgs/MoveBaseAction.h>


class DecisionMaker {
public:
  /**
   *  Class Constructor
   */
  DecisionMaker(ros::NodeHandle nh, ros::NodeHandle nh_);

  /**
   *  Class Destructor
   */
  ~DecisionMaker();

  unsigned int mapToSBPLCost(int occupancy);

  void idToCell(unsigned int id, unsigned int& x, unsigned int& y,
                unsigned int width, unsigned int height);

  void createFootprint(std::vector<sbpl_2Dpt_t>& perimeter, double halfwidth,
                                                            double halflength);

  void planPath(geometry_msgs::Pose2D start_pose, geometry_msgs::Pose2D goal_pose);

  void startExploration();

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Ros msgs
  nav_msgs::OccupancyGrid latest_map_msg_;
  nav_msgs::Path plan_path_;

  // Publisher, subscriber and services
  ros::Publisher plan_pub_;
  ros::Subscriber map_sub_;
  ros::ServiceClient frontier_exploration_client_;
  ros::ServiceClient map_recalculation_client_;

  // others
  unsigned int width_;
  unsigned int height_;
  float resolution_;

  bool finished_;
  bool map_initialized_;
  std::string primitive_filename_;
  SBPLPlanner* planner_;
  EnvironmentNAVXYTHETALAT env_;
};


#endif // DECISION_MAKER_H
