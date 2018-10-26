/**
 *  graph_optimisation.h
 *
 *  Created on: 13.08.2018
 *      Author: Dario Mammolo
 */

#ifndef GRAPH_OPTIMISATION_H
#define GRAPH_OPTIMISATION_H

#include <fstream>
#include <ctime>

#include <ros/ros.h>
#include <ros/package.h>
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
#include <crowdbot_active_slam/service_call.h>
#include <crowdbot_active_slam/utility_calc.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

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
   *  ...
   */
  void getSubsetOfMap(nav_msgs::Path action_path,
                      std::vector<double> alpha,
                      std::map<int, double>& subset);

  /**
   *  ...
   */
  void updateLogOdsWithBresenham(int x0, int y0, int x1, int y1,
                               std::vector<int> end_point_index_m, bool update);

  /**
   *  Calculates map with current factor graph and keyframe scans and draws map.
   */
  void drawMap(gtsam::Values pose_estimates, std::vector<LDP> keyframe_ldp_vec);

  /**
   *  Updates current map with newest scans.
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
  void scanMatcherCallback(const geometry_msgs::Pose2D::ConstPtr& pose2D_msg);

  /**
   *  Service callback for saving current uncertainty matrix along path
   */
  bool saveUncertaintyMatServiceCallback(
    crowdbot_active_slam::service_call::Request &request,
    crowdbot_active_slam::service_call::Response &response);

  /**
   *  Service callback for recalculating the map
   */
  bool mapRecalculationServiceCallback(
    crowdbot_active_slam::service_call::Request &request,
    crowdbot_active_slam::service_call::Response &response);

  /**
   *  Service callback for calculating utility of a plan
   */
  bool utilityCalcServiceCallback(
    crowdbot_active_slam::utility_calc::Request &request,
    crowdbot_active_slam::utility_calc::Response &response);

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Ros msgs
  nav_msgs::Path graph_path_;
  nav_msgs::OccupancyGrid occupancy_grid_msg_;
  sensor_msgs::LaserScan latest_scan_msg_;

  // Service, Publisher and Subscriber
  ros::Subscriber pose_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher path_pub_;
  ros::Publisher action_path_pub_;
  ros::Publisher map_pub_;
  ros::ServiceServer map_service_;
  ros::ServiceServer utility_calc_service_;
  ros::ServiceServer uncertainty_service_;

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
  gtsam::Pose2 last_pose2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::noiseModel::Diagonal::shared_ptr scan_match_noise_;
  gtsam::Values pose_estimates_;
  gtsam::Values new_estimates_;
  gtsam::ISAM2 isam_;

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
  bool const_map_update_steps_;
  bool first_scan_pose_;
  bool scan_callback_initialized_;
  bool new_node_;
  bool first_map_published_;
  ros::Time last_published_;
  int node_counter_;
  unsigned int scan_ranges_size_;
  double scan_angle_increment_;
  std::vector<gtsam::Matrix> uncertainty_matrices_path_;

};

#endif  // GRAPH_OPTIMISATION_H
