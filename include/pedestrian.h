/**
 *  Adapted from:
 *  https://github.com/ethz-asl/deep_interaction_modeling/blob/master/sf_model/
 *  gazebo_sim/include/pedestrian.hpp
 */

#pragma once
#ifndef INCLUDE_PEDESTRIAN_H_
#define INCLUDE_PEDESTRIAN_H_

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <cmath>
#include <deque>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <string>
#include <utils.hpp>

using namespace std;

class Pedestrian {
 public:
  void SetName(std::string name);
  void SetNumber(int number);
  void SetCurrentPose(const nav_msgs::Odometry::ConstPtr& msg);
  void SetGoal(const geometry_msgs::Vector3::ConstPtr& msg);
  void PubGoal(grid_map::GridMap* map);
  void ShowGoal();
  void SetElapsedTime(ros::Time time);

  bool controlled = false;
  const std::string* GetName();
  const int* GetNumber();
  geometry_msgs::Vector3 GetCurrentPose();
  geometry_msgs::Vector3 GetGoal();
  geometry_msgs::Vector3 GetSpeed();
  tf::Quaternion GetOrientation();
  double GetRelativeObjectDistance();
  int GetRelativeObjectAngle();
  deque<geometry_msgs::Vector3> GetPoseHistory();
  ros::Time GetElapsedTime();

  ros::Subscriber rosSub_odom;
  ros::Subscriber rosSub_goal;
  ros::Publisher rosPub_cmd_vel;
  ros::Publisher rosPub_goal;

  ros::ServiceClient* gazebo_spawn_goal;
  ros::ServiceClient* gazebo_delete_goal;

 private:
  int ped_number;
  double distance_of_obj;
  int angle_of_obj;
  bool spawned_goal = false;
  ros::Time elapsed_time;
  std::string name;
  deque<geometry_msgs::Vector3>* pose_hist =
      new deque<geometry_msgs::Vector3>(4);
  geometry_msgs::Vector3 current_pose;
  geometry_msgs::Vector3 current_speed;
  geometry_msgs::Vector3 desired_pose;
  tf::Quaternion current_orientation;
};
#endif  // INCLUDE_PEDESTRIAN_H_
