/**
 *  static_laser_scan_combiner.h
 *
 *  Created on: 07.12.2018
 *      Author: Dario Mammolo
 */

#ifndef STATIC_SCAN_EXTRACTOR_H
#define STATIC_SCAN_EXTRACTOR_H

#include <ros/ros.h>
#include <object_detector.h>
#include <tracked_object.h>
#include <kalman_filter.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <crowdbot_active_slam/current_pose.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

#include <csm/csm_all.h>
#undef min
#undef max

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicy;


class StaticScanExtractor{
public:
  /**
   *  Class Constructor
   */
  StaticScanExtractor(ros::NodeHandle nh, ros::NodeHandle nh_);

  /**
   *  Class Destructor
   */
  ~StaticScanExtractor();

  /**
   *  Transforms laser scans to the LDP data form, which is needed for scan
   *  matching with csm.
   */
  void laserScanToLDP(sensor_msgs::LaserScan& scan_msg, LDP& ldp);

  /**
   *  The initialisation function for generating the first static scan for the
   *  SLAM framework. It is summing up laser scans over time and choses the
   *  scans which are most probable to be static.
   */
  void initScan(sensor_msgs::LaserScan& laser_msg,
                sensor_msgs::LaserScan& init_scan);

  /**
   *  The main function which recives the raw laser scan and publishes then the
   *  extracted static laser scan for the SLAM framework.
   */
  void scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                        const nav_msgs::Odometry::ConstPtr& odom_msg);

  /**
   *  A callback for the occupancy grid map from the SLAM framework.
   */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Services
  ros::ServiceClient get_current_pose_client_;

  // Publisher, Subscriber
  ros::Subscriber map_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> *scan_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
  message_filters::Synchronizer<SyncPolicy> *sync_;

  ros::Publisher static_scan_pub_;
  ros::Publisher dynamic_scan_pub_;
  ros::Publisher moving_objects_pub_;
  ros::Publisher static_objects_pub_;
  ros::Publisher unknown_objects_pub_;

  // ROS msgs
  sensor_msgs::LaserScan init_scan_;
  sensor_msgs::LaserScan static_scan_;
  sensor_msgs::LaserScan laser_msg_;
  sensor_msgs::LaserScan dynamic_scan_;
  sensor_msgs::LaserScan node_scan_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::OccupancyGrid map_msg_;

  // CSM
  sm_params sm_icp_params_;
  sm_result sm_icp_result_;
  LDP prev_node_ldp_;
  LDP current_ldp_;
  unsigned int scan_ranges_size_;
  double scan_angle_increment_;
  double scan_range_min_;
  double scan_range_max_;

  // TF
  tf::TransformListener base_to_laser_listener_;
  tf::Transform base_to_laser_;
  tf::Transform laser_to_base_;
  tf::Stamped<tf::Transform> current_pose_tf_;
  tf::Transform current_odom_tf_;
  tf::Transform map_to_odom_tf_;
  tf::Transform map_to_last_node_tf_;
  tf::Transform map_to_latest_tf_;
  tf::Transform map_to_latest_laser_tf_;

  // Objects
  ObjectDetector object_detector_;

  // variables
  bool initialized_first_scan_;
  std::vector<std::vector<double>> init_scan_sum_;
  ros::Time begin_time_;
  ros::Time prev_node_stamp_;
  ros::Time curr_node_stamp_;
  double prev_delta_t_;
  double std_dev_process_;
  double std_dev_range_;
  double std_dev_theta_;
  std::vector<geometry_msgs::Point> occluded_means_;
  std::vector<geometry_msgs::Point> free_means_;
  std::vector<TrackedObject> tracked_objects_;
  KalmanFilter kalman_filter_;
  bool map_callback_initialized_;
};

#endif  // STATIC_SCAN_EXTRACTOR_H
