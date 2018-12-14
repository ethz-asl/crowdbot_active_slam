/**
 *  static_laser_scan_combiner.h
 *
 *  Created on: 07.12.2018
 *      Author: Dario Mammolo
 */

#ifndef STATIC_LASER_SCAN_COMBINER_H
#define STATIC_LASER_SCAN_COMBINER_H

#include <ros/ros.h>
#include <object_detector.h>
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

typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::LaserScan, nav_msgs::Odometry> SyncPolicy;

class StaticLaserScanCombiner{
public:
  /**
   *  Class Constructor
   */
  StaticLaserScanCombiner(ros::NodeHandle nh, ros::NodeHandle nh_);

   /**
    *  Class Destructor
    */
  ~StaticLaserScanCombiner();

  void initScan(sensor_msgs::LaserScan& laser_msg,
                sensor_msgs::LaserScan& init_scan);

  void scanOdomCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                        const nav_msgs::Odometry::ConstPtr& odom_msg);

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
  ros::Publisher marker_front_pub_;
  ros::Publisher marker_occluded_pub_;

  // ROS msgs
  sensor_msgs::LaserScan init_scan_;
  sensor_msgs::LaserScan laser_msg_;
  sensor_msgs::LaserScan dynamic_scan_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::OccupancyGrid::ConstPtr map_msg_;

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
  ros::Time last_node_stamp_;
};

#endif  // STATIC_LASER_SCAN_COMBINER_H
