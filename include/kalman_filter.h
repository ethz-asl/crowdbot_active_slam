/**
 *  kalman_filter.cpp
 *
 *  Created on: 17.12.2018
 *      Author: Dario Mammolo
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <ros/ros.h>
#include <Eigen/Dense>


class KalmanFilter{
public:
  /**
   *  Class Constructor
   */
  KalmanFilter();

  /**
   *  Class Constructor
   */
  KalmanFilter(double std_dev_range, double std_dev_theta);

  /**
   *  Class Destructor
   */
  ~KalmanFilter();

  void updateProcessMatrices(double dt);

  Eigen::Vector4d prediction(Eigen::Vector4d prev_mean);
  Eigen::Matrix4d prediction(Eigen::Matrix4d prev_var);

  void updateKalmanGain(Eigen::Matrix4d prev_var);

  Eigen::Vector4d update(Eigen::Vector4d prev_mean, Eigen::Vector2d z_m);
  Eigen::Matrix4d update(Eigen::Matrix4d prev_var, Eigen::Vector2d z_m);

private:
  Eigen::Matrix4d A_prev_;
  Eigen::Matrix4d Q_prev_;
  Eigen::MatrixXd H_prev_;
  Eigen::Matrix2d R_prev_;
  Eigen::MatrixXd kalman_gain_;
};

#endif  // KALMAN_FILTER_H
