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

  // bool check_if_dynamic();

  // Variables
  int counter_not_seen;
  Eigen::Vector4d state_mean;
  Eigen::Matrix4d state_var;



private:
  // State
  double x_, y_, x_vel_, y_vel_;
  std::string current_occlusion_;
  std::string dynamic_or_static_;

};

#endif  // TRACKED_OBJECT_H
