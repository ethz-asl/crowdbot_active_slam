/**
 *  kalman_filter.cpp
 *
 *  Created on: 17.12.2018
 *      Author: Dario Mammolo
 */

#include <kalman_filter.h>

KalmanFilter::KalmanFilter(){}


KalmanFilter::KalmanFilter(double std_dev_range, double std_dev_theta){
  // Measurement matrices
  // Currently using easy approach and std dev are for x and y
  // Init H_prev
  H_prev_ = Eigen::MatrixXd(2, 4);
  H_prev_ << 1, 0, 0, 0,
             0, 1, 0, 0;

  // Init R_prev
  R_prev_ << std::pow(std_dev_range, 2), 0,
             0, std::pow(std_dev_theta, 2);

  // Init A_prev with random dt
  A_prev_ << 1, 0, 0.1, 0,
            0, 1, 0, 0.1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // Init Q_prev with random dt
  Q_prev_ << 0.1, 0, 0.1, 0,
            0, 0.1, 0, 0.1,
            0.1, 0, 0.1, 0,
            0, 0.1, 0, 0.1;

  // Init kalman gain
  kalman_gain_ = Eigen::MatrixXd(4, 2);
}

KalmanFilter::~KalmanFilter(){}

void KalmanFilter::updateProcessMatrices(double dt){
  // Update A_prev
  A_prev_(0, 2) = dt;
  A_prev_(1, 3) = dt;

  // Update Q_prev
  Q_prev_(0, 0) = std::pow(dt, 4) / 4;
  Q_prev_(0, 2) = std::pow(dt, 3) / 2;
  Q_prev_(1, 1) = std::pow(dt, 4) / 4;
  Q_prev_(1, 3) = std::pow(dt, 3) / 2;
  Q_prev_(2, 0) = std::pow(dt, 3) / 2;
  Q_prev_(2, 2) = std::pow(dt, 2);
  Q_prev_(3, 1) = std::pow(dt, 3) / 2;
  Q_prev_(3, 3) = std::pow(dt, 2);
}

Eigen::Vector4d KalmanFilter::prediction(Eigen::Vector4d prev_mean){
  Eigen::Vector4d state_mean = A_prev_ * prev_mean;
  return state_mean;
}

Eigen::Matrix4d KalmanFilter::prediction(Eigen::Matrix4d prev_var){
  Eigen::Matrix4d state_var = A_prev_ * prev_var * A_prev_.transpose() + Q_prev_;
  return state_var;
}

void KalmanFilter::updateKalmanGain(Eigen::Matrix4d prev_var){
  kalman_gain_ = prev_var * H_prev_.transpose() *
                (H_prev_ * prev_var * H_prev_.transpose() + R_prev_).inverse();
}


Eigen::Vector4d KalmanFilter::update(Eigen::Vector4d prev_mean, Eigen::Vector2d z_m){
  Eigen::Vector4d state_mean;
  state_mean = prev_mean + kalman_gain_ * (z_m - H_prev_ * prev_mean);
  return state_mean;
}

Eigen::Matrix4d KalmanFilter::update(Eigen::Matrix4d prev_var, Eigen::Vector2d z_m){
  Eigen::Matrix4d state_var;
  state_var = (Eigen::MatrixXd::Identity(4, 4) - kalman_gain_ * H_prev_) * prev_var;
  return state_var;
}
