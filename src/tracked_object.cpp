/**
 *  tracked_object.cpp
 *
 *  Created on: 17.12.2018
 *      Author: Dario Mammolo
 */

 #include <tracked_object.h>

TrackedObject::TrackedObject(double x, double y){
  // Init state mean
  x_ = x;
  y_ = y;
  x_vel_ = 0;
  y_vel_ = 0;
  x_acc_ = 0;
  y_acc_ = 0;
  state_mean << x_, y_, x_vel_, y_vel_, x_acc_, y_acc_;

  // Init state var
  state_var << 1, 0, 0, 0, 0, 0,
               0, 1, 0, 0, 0, 0,
               0, 0, 1, 0, 0, 0,
               0, 0, 0, 1, 0, 0,
               0, 0, 0, 0, 1, 0,
               0, 0, 0, 0, 0, 1;

  // Init counter for not seen
  counter_not_seen = 0;

  velocity_counter = 0;
  x_vel_sum = 0;
  y_vel_sum = 0;
  x_vel_av = 0;
  y_vel_av = 0;
}

TrackedObject::~TrackedObject(){}

void TrackedObject::saveCluster(std::vector<geometry_msgs::Point> cluster){
  if (prev_cluster_.empty()){
    prev_cluster_ = cluster;
    current_cluster_ = cluster;
  }
  else {
    prev_cluster_ = current_cluster_;
    current_cluster_ = cluster;
  }
}

void TrackedObject::computeAverageSpeed(int averaging_size){
  x_vel_vec_.push_back(state_mean[2]);
  y_vel_vec_.push_back(state_mean[3]);
  // Pop first element
  if (x_vel_vec_.size() > averaging_size){
    x_vel_vec_.erase(x_vel_vec_.begin());
    y_vel_vec_.erase(y_vel_vec_.begin());
  }

  x_vel_av = 0;
  y_vel_av = 0;
  for (size_t i = 0; i < x_vel_vec_.size(); i++){
    x_vel_av += x_vel_vec_[i];
    y_vel_av += y_vel_vec_[i];
  }
  x_vel_av /= x_vel_vec_.size();
  y_vel_av /= x_vel_vec_.size();
}
