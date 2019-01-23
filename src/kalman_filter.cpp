/**
 *  kalman_filter.cpp
 *
 *  Created on: 17.12.2018
 *      Author: Dario Mammolo
 */

#include <kalman_filter.h>

KalmanFilter::KalmanFilter(){}


KalmanFilter::KalmanFilter(double std_dev_range, double std_dev_theta, double std_dev_process){
  // Measurement matrices
  // Currently using easy approach and std dev are for x and y
  // Init H_prev
  H_prev_ = Eigen::MatrixXd(2, 6);
  H_prev_ << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0;

  // Init R_prev
  R_prev_ << std::pow(std_dev_range, 2), 0,
             0, std::pow(std_dev_theta, 2);

  // Init A_prev with random dt
  A_prev_ << 1, 0, 0.067, 0, 0.067 * 0.067 / 2, 0,
             0, 1, 0.067, 0, 0,                 0.067 * 0.067 / 2,
             0, 0, 1,     0, 0.067,             0,
             0, 0, 0,     1, 0,                 0.067,
             0, 0, 0,     0, 1,                 0,
             0, 0, 0,     0, 0,                 1;

  // Init Q_prev with random dt
  Q_prev_ << std::pow(0.067, 4) / 4, 0, std::pow(0.067, 3) / 2, 0, std::pow(0.067, 2) / 2, 0,
            0, std::pow(0.067, 4) / 4, 0, std::pow(0.067, 3) / 2, 0,  std::pow(0.067, 2) / 2,
            std::pow(0.067, 3) / 2, 0, std::pow(0.067, 2), 0, 0.067, 0,
            0, std::pow(0.067, 3) / 2, 0, std::pow(0.067, 2), 0, 0.067,
            std::pow(0.067, 2) / 2, 0, 0.067, 0, 1, 0,
            0, std::pow(0.067, 2) / 2, 0, 0.067, 0, 1;

  std_dev_process_ = std_dev_process;
  std_dev_process_sq_ = std_dev_process_* std_dev_process_;

  Q_prev_ = std_dev_process_sq_ * Q_prev_;

  // Init kalman gain
  kalman_gain_ = Eigen::MatrixXd(6, 2);
}

KalmanFilter::~KalmanFilter(){}

void KalmanFilter::updateProcessMatrices(double dt){
  // Update A_prev
  A_prev_(0, 2) = dt;
  A_prev_(0, 4) = dt * dt / 2;
  A_prev_(1, 3) = dt;
  A_prev_(1, 5) = dt * dt / 2;
  A_prev_(2, 4) = dt;
  A_prev_(3, 5) = dt;

  // Update Q_prev
  Q_prev_(0, 0) = std::pow(dt, 4) / 4;
  Q_prev_(0, 2) = std::pow(dt, 3) / 2;
  Q_prev_(0, 4) = std::pow(dt, 2) / 2;
  Q_prev_(1, 1) = Q_prev_(0, 0);
  Q_prev_(1, 3) = Q_prev_(0, 2);
  Q_prev_(1, 5) = Q_prev_(0, 4);
  Q_prev_(2, 0) = Q_prev_(0, 2);
  Q_prev_(2, 2) = std::pow(dt, 2);
  Q_prev_(2, 4) = dt;
  Q_prev_(3, 1) = Q_prev_(0, 2);
  Q_prev_(3, 3) = Q_prev_(2, 2);
  Q_prev_(3, 5) = dt;
  Q_prev_(4, 0) = Q_prev_(0, 4);
  Q_prev_(4, 2) = dt;
  Q_prev_(5, 1) = Q_prev_(0, 4);
  Q_prev_(5, 3) = dt;
}

Vector6d KalmanFilter::prediction(Vector6d prev_mean){
  Vector6d state_mean = A_prev_ * prev_mean;
  return state_mean;
}

Matrix6d KalmanFilter::prediction(Matrix6d prev_var){
  Matrix6d state_var = A_prev_ * prev_var * A_prev_.transpose() + Q_prev_;
  return state_var;
}

void KalmanFilter::updateKalmanGain(Matrix6d prev_var){
  kalman_gain_ = prev_var * H_prev_.transpose() *
                (H_prev_ * prev_var * H_prev_.transpose() + R_prev_).inverse();
}


Vector6d KalmanFilter::update(Vector6d prev_mean, Eigen::Vector2d z_m){
  Vector6d state_mean;
  state_mean = prev_mean + kalman_gain_ * (z_m - H_prev_ * prev_mean);
  return state_mean;
}

Matrix6d KalmanFilter::update(Matrix6d prev_var, Eigen::Vector2d z_m){
  Matrix6d state_var;
  state_var = (Eigen::MatrixXd::Identity(6, 6) - kalman_gain_ * H_prev_) * prev_var;
  return state_var;
}
