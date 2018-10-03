#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

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

  void start_SLAM();

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Services
  ros::ServiceClient frontier_exploration_client_;
  ros::ServiceClient map_recalculation_client_;

  // others
  bool finished_;
};


#endif // DECISION_MAKER_H
