/**
 *  tracked_object.h
 *
 *  Created on: 17.12.2018
 *      Author: Dario Mammolo
 */

#ifndef TRACKED_OBJECT_H
#define TRACKED_OBJECT_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>


class TrackedObject{
public:
  /**
   *  Class Constructor
   */
  TrackedObject(double x, double y);

  /**
   *  Class Destructor
   */
  ~TrackedObject();

  void saveCluster(std::vector<geometry_msgs::Point> cluster);

  // bool check_if_dynamic();

  // Variables
  int counter_not_seen;
  Eigen::Vector4d state_mean;
  Eigen::Matrix4d state_var;
  std::string current_occlusion;
  std::string dynamic_or_static;
  int unknown_since;

private:
  // State
  double x_, y_, x_vel_, y_vel_;
  std::vector<geometry_msgs::Point> prev_cluster_;
  std::vector<geometry_msgs::Point> current_cluster_;

};

#endif  // TRACKED_OBJECT_H
