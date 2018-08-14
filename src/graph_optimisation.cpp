// created: 13.08.2018
// author: Dario Mammolo

#include <graph_optimisation.h>

using namespace gtsam;

GraphOptimiser::GraphOptimiser(ros::NodeHandle nh):
  nh_(nh)
{
  ROS_INFO("Started GraphOptimiser");
  initParams();
  pose_sub_ = nh_.subscribe("pose2D", 1, &GraphOptimiser::scanMatcherCallback, this);
}

GraphOptimiser::~GraphOptimiser(){}

void GraphOptimiser::initParams(){
  nh_.param<double>("node_dist_linear", node_dist_linear_, 0.10);
  nh_.param<double>("node_dist_angular", node_dist_angular_, 0.175);
  dist_linear_sq_ = node_dist_linear_ * node_dist_linear_;
  first_scan_pose_ = true;
  node_counter_ = 0;
  scan_match_noise_ = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, 0.05));
  path_pub_ = nh_.advertise<nav_msgs::Path>("/graph_path", 1);
}

geometry_msgs::PoseStamped GraphOptimiser::createPoseStamped(Pose2 pose2){
  geometry_msgs::Point pose_position;
  pose_position.x = pose2.x();
  pose_position.y = pose2.y();

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pose2.theta());
  geometry_msgs::Quaternion pose_orientation;
  quaternionTFToMsg(q, pose_orientation);

  geometry_msgs::Pose pose;
  pose.position = pose_position;
  pose.orientation = pose_orientation;

  geometry_msgs::PoseStamped posestamped;
  posestamped.pose = pose;

  return posestamped;
}

void GraphOptimiser::scanMatcherCallback(const geometry_msgs::Pose2D::ConstPtr& pose2D_msg){
  current_pose2_ = Pose2((*pose2D_msg).x, (*pose2D_msg).y, (*pose2D_msg).theta);
  if (first_scan_pose_){
    pose_estimates_.insert(node_counter_, current_pose2_);
    noiseModel::Diagonal::shared_ptr prior_noise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.01));
    graph_.add(PriorFactor<Pose2>(node_counter_, current_pose2_, prior_noise));
    prev_pose2_ = current_pose2_;

    geometry_msgs::PoseStamped posestamped =
    createPoseStamped(current_pose2_);
    graph_path_.poses.push_back(posestamped);

    first_scan_pose_ = false;
    node_counter_ += 1;
    return;
  }

  double x_diff = current_pose2_.x() - prev_pose2_.x();
  double y_diff = current_pose2_.y() - prev_pose2_.y();
  double diff_dist_linear_sq = x_diff * x_diff + y_diff * y_diff;
  double angle_diff = current_pose2_.theta() - prev_pose2_.theta();

  if ((diff_dist_linear_sq < dist_linear_sq_) &&
     (std::abs(angle_diff) < node_dist_angular_)) return;

  pose_estimates_.insert(node_counter_, current_pose2_);
  Pose2 next_mean = prev_pose2_.between(current_pose2_);
  graph_.add(BetweenFactor<Pose2>(node_counter_ - 1, node_counter_,
    next_mean, scan_match_noise_));

  LevenbergMarquardtOptimizer optimizer(graph_, pose_estimates_);
  ROS_INFO("Optimisation started!");
  pose_estimates_ = optimizer.optimize();
  ROS_INFO("Optimisation finished!");
  pose_estimates_.print("Pose estimates:\n");

  nav_msgs::Path new_path;
  graph_path_ = new_path;
  graph_path_.header.frame_id = "/odom";
  Pose2 tmp_pose2;
  geometry_msgs::PoseStamped tmp_posestamped;
  for (int i = 0; i < pose_estimates_.size(); i++){
    tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));
    tmp_posestamped = createPoseStamped(tmp_pose2);
    graph_path_.poses.push_back(tmp_posestamped);
  }

  path_pub_.publish(graph_path_);
  node_counter_ += 1;
  prev_pose2_ = current_pose2_;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "GraphOptimiser");
  ros::NodeHandle nh;
  GraphOptimiser optimiser(nh);
  ros::spin();
  return 0;
}
