/**
 *  remove_people_from_pcl.h
 *
 *  Created on: 14.05.2019
 *      Author: Dario Mammolo
 */

#ifndef REMOVE_PEOPLE_FROM_PCL_H
#define REMOVE_PEOPLE_FROM_PCL_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_converter/ObstacleMsg.h>
#include <costmap_converter/ObstacleArrayMsg.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
typedef message_filters::sync_policies::ApproximateTime
  <sensor_msgs::PointCloud2, costmap_converter::ObstacleArrayMsg> SyncPolicy;


class RemovePeopleFromPCL
{
public:
  /**
   *  Class Constructor
   */
  RemovePeopleFromPCL(ros::NodeHandle nh, ros::NodeHandle nh_);

  /**
   *  Class Destructor
   */
  ~RemovePeopleFromPCL();

  /**
   *  A PointCloud2/pcl callback
   */
  void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg,
                   const costmap_converter::ObstacleArrayMsg::ConstPtr& obstacle_msg);

private:
  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Time syncronizer subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> *pcl_sub_;
  message_filters::Subscriber<costmap_converter::ObstacleArrayMsg> *obstacle_sub_;
  message_filters::Synchronizer<SyncPolicy> *sync_;

  // Publisher
  ros::Publisher removed_pcl_pub_;

  // Parameters
  std::string pcl_callback_topic_;
  std::string obstacle_callback_topic_;
  float max_dist_from_robot_;
  float sq_max_dist_from_robot_;

  // TF
  tf::TransformListener map_to_cam_listener_;

};

#endif // REMOVE_PEOPLE_FROM_PCL_H
