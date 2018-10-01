/**
 *  graph_optimisation.h
 *
 *  Created on: 13.08.2018
 *      Author: Dario Mammolo
 */

#ifndef GRAPH_OPTIMISATION_H
#define GRAPH_OPTIMISATION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>

#include <csm/csm_all.h>
#undef min
#undef max


class GraphOptimiser{
public:
  /**
   *  Class Constructor
   */
  GraphOptimiser(ros::NodeHandle nh, ros::NodeHandle nh_);

  /**
   *  Class Destructor
   */
  ~GraphOptimiser();

  /**
   *  Helper function which initialises some parameters
   */
  void initParams();

  /**
   *  A helper function which creates LDP from laser scans.
   */
  void laserScanToLDP(sensor_msgs::LaserScan& scan_msg, LDP& ldp);

  /**
   *  Calculates map with current factor graph and keyframe scans and draws map.
   */
  void drawMap(gtsam::Values pose_estimates, std::vector<LDP> keyframe_ldp_vec);

  /**
   *  Updates current map with new scans.
   */
  void updateMap(gtsam::Values pose_estimates,
                 std::vector<LDP> keyframe_ldp_vec);

  /**
   *  A callback function on laser scans.
   */
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  /**
   *  A callback function on Pose2D scans from the laser scan matcher
   *  which does Graph construction and optimisation.
   */
  void scanMatcherCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg);

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Ros msgs
  nav_msgs::Path graph_path_;
  sensor_msgs::LaserScan latest_scan_msg_;

  // Publisher and Subscriber
  ros::Subscriber pose_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher path_pub_;
  ros::Publisher map_pub_;

  // TF
  tf::Transform map_to_odom_tf_;
  tf::TransformBroadcaster map_br_;
  tf::TransformListener odom_listener_;
  tf::TransformListener base_to_laser_listener_;
  tf::Transform base_to_laser_;
  tf::Transform laser_to_base_;

  // gtsam objects
  gtsam::Pose2 prev_pose2_;
  gtsam::Pose2 current_pose2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::noiseModel::Diagonal::shared_ptr scan_match_noise_;
  gtsam::Values pose_estimates_;

  // CSM
  sm_params sm_icp_params_;
  sm_result sm_icp_result_;
  LDP current_ldp_;
  std::vector<LDP> keyframe_ldp_vec_;

  // Map parameters
  float map_resolution_;
  unsigned int map_width_;
  unsigned int map_height_;
  Eigen::MatrixXf log_odds_array_;
  float l_0_;
  float p_occ_;
  float p_free_;
  float l_occ_;
  float l_free_;

  // Other variables
  double node_dist_linear_;
  double node_dist_angular_;
  double dist_linear_sq_;
  double lc_radius_;
  bool first_scan_pose_;
  bool scan_callback_initialized_;
  int node_counter_;

};

#endif  // GRAPH_OPTIMISATION_H
