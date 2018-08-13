// created: 13.08.2018
// author: Dario Mammolo

#include <graph_optimisation.h>

using namespace gtsam;

GraphOptimiser::GraphOptimiser(ros::NodeHandle nh):
  nh_(nh)
{
  ROS_INFO("Started GraphOptimiser");
  initParams();
  pose_sub = nh_.subscribe("pose2D", 1, &GraphOptimiser::scanMatcherCallback, this);
}

GraphOptimiser::~GraphOptimiser(){}

void GraphOptimiser::initParams(){
  nh_.param<double>("node_dist_linear", node_dist_linear_, 0.10);
  nh_.param<double>("node_dist_angular", node_dist_angular_, 0.175);
  dist_linear_sq_ = node_dist_linear_ * node_dist_linear_;
  first_scan_pose_ = true;
  node_counter_ = 0;
  scan_match_noise_ = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, 0.05));
}

void GraphOptimiser::scanMatcherCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg){
  current_pose_ = *pose_msg;
  if (first_scan_pose_){
    Pose2 prior_mean(0.0, 0.0, 0.0);
    pose_estimates_.insert(node_counter_, Pose2(0.0, 0.0, 0.0));
    noiseModel::Diagonal::shared_ptr prior_noise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.01));
    graph_.add(PriorFactor<Pose2>(node_counter_, prior_mean, prior_noise));
    prev_pose_ = current_pose_;
    first_scan_pose_ = false;
    node_counter_ += 1;
    return;
  }

  double x_diff = current_pose_.x - prev_pose_.x;
  double y_diff = current_pose_.y - prev_pose_.y;
  double diff_dist_linear_sq = x_diff * x_diff + y_diff * y_diff;
  double angle_diff = current_pose_.theta - prev_pose_.theta;

  if (diff_dist_linear_sq < dist_linear_sq_) return;
  if (std::abs(angle_diff) < node_dist_angular_) return;

  pose_estimates_.insert(node_counter_,
                  Pose2(current_pose_.x, current_pose_.y, current_pose_.theta));
  Pose2 next_mean(x_diff, y_diff, angle_diff);
  graph_.add(BetweenFactor<Pose2>(node_counter_ - 1, node_counter_,
    next_mean, scan_match_noise_));

  LevenbergMarquardtOptimizer optimizer(graph_, pose_estimates_);
  pose_estimates_ = optimizer.optimize();

  node_counter_ += 1;
  prev_pose_ = current_pose_;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "GraphOptimiser");
  ros::NodeHandle nh;
  GraphOptimiser optimiser(nh);
  ros::spin();
  return 0;
}
