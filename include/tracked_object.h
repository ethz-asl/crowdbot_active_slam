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

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

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

  void computeAverageSpeed(int averaging_size);

  // bool check_if_dynamic();

  // Variables
  int counter_not_seen;
  Vector6d state_mean;
  Matrix6d state_var;
  std::string current_occlusion;
  std::string dynamic_or_static;
  int unknown_since;
  int unknown_since_ever;
  int velocity_counter;
  double x_vel_sum;
  double y_vel_sum;
  double x_vel_av;
  double y_vel_av;

private:
  // State
  double x_, y_, x_vel_, y_vel_, x_acc_, y_acc_;
  std::vector<double> x_vel_vec_;
  std::vector<double> y_vel_vec_;
  std::vector<geometry_msgs::Point> prev_cluster_;
  std::vector<geometry_msgs::Point> current_cluster_;

};

#endif  // TRACKED_OBJECT_H
