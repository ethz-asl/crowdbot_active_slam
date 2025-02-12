#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

#include <fstream>
#include <ctime>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

#include <crowdbot_active_slam/service_call.h>
#include <crowdbot_active_slam/get_frontier_list.h>
#include <crowdbot_active_slam/utility_calc.h>
#include <std_srvs/Empty.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <crowdbot_active_slam/get_map.h>
#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalID.h>


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

  /**
   *  Calculate cell ids from map id
   */
  void idToCell(unsigned int id, unsigned int& x, unsigned int& y,
                int width, int height);

  /**
   *  Function which starts the exploration task when starting or ariving at goal
   */
  void startExploration();

  /**
   *  Save map in txt file sich that it can be used for calculations later
   */
   void saveGridMap();

   /**
    *  Save general information to the exploration
    */
   void saveGeneralResults();

   /**
    *  Callback for joystick msgs (when using real pepper). Emergency stop if
    *  pepper should do something unintended. Cancels current move base goal and
    *  shuts down node.
    */
   void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Ros msgs
  nav_msgs::OccupancyGrid latest_map_msg_;
  nav_msgs::Path plan_path_;

  // Publisher, subscriber and services
  ros::Publisher plan_pub_;
  ros::Publisher cancel_move_base_pub_;
  ros::Subscriber map_sub_;
  ros::Subscriber joy_sub_;
  ros::ServiceClient frontier_exploration_client_;
  ros::ServiceClient map_recalculation_client_;
  ros::ServiceClient utility_calc_client_;
  ros::ServiceClient get_plan_move_base_client_;
  ros::ServiceClient uncertainty_client_;
  ros::ServiceClient get_map_client_;
  ros::ServiceClient clear_costmap_client_;

  // ros params
  int map_width_;
  int map_height_;
  float map_resolution_;
  double node_dist_linear_;
  double node_dist_angular_;
  double lc_radius_;

  std::string exploration_type_;
  ros::Time start_time_;
  ros::Time end_time_;
  std::string save_directory_path_;
  bool return_to_start_;
};


#endif // DECISION_MAKER_H
