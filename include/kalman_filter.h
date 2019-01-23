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

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class KalmanFilter{
public:
  /**
   *  Class Constructor
   */
  KalmanFilter();

  /**
   *  Class Constructor
   */
  KalmanFilter(double std_dev_range, double std_dev_theta, double std_dev_process);

  /**
   *  Class Destructor
   */
  ~KalmanFilter();

  void updateProcessMatrices(double dt);

  Vector6d prediction(Vector6d prev_mean);
  Matrix6d prediction(Matrix6d prev_var);

  void updateKalmanGain(Matrix6d prev_var);

  Vector6d update(Vector6d prev_mean, Eigen::Vector2d z_m);
  Matrix6d update(Matrix6d prev_var, Eigen::Vector2d z_m);

private:
  double std_dev_process_;
  double std_dev_process_sq_;
  Matrix6d A_prev_;
  Matrix6d Q_prev_;
  Eigen::MatrixXd H_prev_;
  Eigen::Matrix2d R_prev_;
  Eigen::MatrixXd kalman_gain_;
};

#endif  // KALMAN_FILTER_H
