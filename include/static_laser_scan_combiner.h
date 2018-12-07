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
#include <visualization_msgs/Marker.h>


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

   void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publisher, Subscriber
  ros::Publisher static_scan_pub_;
  ros::Publisher marker_front_pub_;
  ros::Publisher marker_occluded_pub_;
  ros::Subscriber scan_sub_;

  // ROS msgs
  sensor_msgs::LaserScan laser_msg_;

  //
  ObjectDetector object_detector_;
};

#endif  // STATIC_LASER_SCAN_COMBINER_H
