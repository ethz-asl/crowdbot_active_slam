/**
 *  graph_optimisation.cpp
 *
 *  Created on: 13.08.2018
 *      Author: Dario Mammolo
 */

#include <graph_optimisation.h>

using namespace gtsam;

GraphOptimiser::GraphOptimiser(ros::NodeHandle nh, ros::NodeHandle nh_):
  nh_(nh), nh_private_(nh_)
{
  ROS_INFO("Started GraphOptimiser");
  initParams();

  // Initialise map to odom tf
  tf::Transform init_tf;
  init_tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion init_q;
  init_q.setRPY(0.0, 0.0, 0);
  init_tf.setRotation(init_q);
  map_to_odom_tf_ = init_tf;
  map_br_.sendTransform(tf::StampedTransform(map_to_odom_tf_, ros::Time::now(),
                        "map", "odom"));
}

GraphOptimiser::~GraphOptimiser(){}

void GraphOptimiser::initParams(){
  // Get params which describe in which distance the node of the graph is built
  nh_private_.param<double>("node_dist_linear", node_dist_linear_, 0.10);
  nh_private_.param<double>("node_dist_angular", node_dist_angular_, 0.175);

  // Initialise subscriber and publisher
  pose_sub_ = nh_.subscribe("pose2D", 1, &GraphOptimiser::scanMatcherCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/graph_path", 1);

  // Initialise noise on scan matching nodes
  scan_match_noise_ = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, 0.05));

  // Initialise other parameters
  dist_linear_sq_ = node_dist_linear_ * node_dist_linear_;
  first_scan_pose_ = true;
  node_counter_ = 0;
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
  // Get newest transform from odom to base_link for later use
  tf::StampedTransform odom_stamped_transform;
  odom_listener_.lookupTransform("/odom", "/base_link", ros::Time(0),
                                 odom_stamped_transform);
  tf::Transform odom_transform;
  odom_transform = static_cast<tf::Transform>(odom_stamped_transform);

  // Save msg as Pose2
  current_pose2_ = Pose2((*pose2D_msg).x, (*pose2D_msg).y, (*pose2D_msg).theta);

  // Check if it is the first scan
  if (first_scan_pose_){
    // Create graph with first node
    pose_estimates_.insert(node_counter_, current_pose2_);
    noiseModel::Diagonal::shared_ptr prior_noise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.01));
    graph_.add(PriorFactor<Pose2>(node_counter_, current_pose2_, prior_noise));

    // Update variables
    first_scan_pose_ = false;
    prev_pose2_ = current_pose2_;
    node_counter_ += 1;
    return;
  }

  // Calculate some variables for decision if to add a new node
  double x_diff = current_pose2_.x() - prev_pose2_.x();
  double y_diff = current_pose2_.y() - prev_pose2_.y();
  double diff_dist_linear_sq = x_diff * x_diff + y_diff * y_diff;
  double angle_diff = current_pose2_.theta() - prev_pose2_.theta();

  // Add a new node if robot moved far enough
  if ((diff_dist_linear_sq > dist_linear_sq_) or
     (std::abs(angle_diff) > node_dist_angular_)){
    // Add new node to graph
    pose_estimates_.insert(node_counter_, current_pose2_);
    Pose2 next_mean = prev_pose2_.between(current_pose2_);
    graph_.add(BetweenFactor<Pose2>(node_counter_ - 1, node_counter_,
      next_mean, scan_match_noise_));

    // Optimize the graph
    LevenbergMarquardtOptimizer optimizer(graph_, pose_estimates_);
    ROS_INFO("Optimisation started!");
    pose_estimates_ = optimizer.optimize();
    ROS_INFO("Optimisation finished!");
    pose_estimates_.print("Pose estimates:\n");

    // Create a path msg of the graph node estimates
    nav_msgs::Path new_path;
    graph_path_ = new_path;
    graph_path_.header.frame_id = "/map";
    Pose2 tmp_pose2;
    geometry_msgs::PoseStamped tmp_posestamped;

    // Iterate over all node estimates
    for (int i = 0; i < pose_estimates_.size(); i++){
      // Cast Pose2 from Value
      tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));

      // Create PoseStamped variables from Pose 2 and add to path
      tmp_posestamped = createPoseStamped(tmp_pose2);
      graph_path_.poses.push_back(tmp_posestamped);
    }

    Pose2 last_pose2;
    last_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(node_counter_));

    tf::Transform last_pose_tf;
    last_pose_tf.setOrigin(tf::Vector3(last_pose2.x(), last_pose2.y(), 0.0));

    tf::Quaternion last_pose_q;
    last_pose_q.setRPY(0.0, 0.0, last_pose2.theta());
    last_pose_tf.setRotation(last_pose_q);

    map_to_odom_tf_ = last_pose_tf * odom_transform.inverse();

    // Publish the graph
    path_pub_.publish(graph_path_);

    // Update variables
    node_counter_ += 1;
    prev_pose2_ = current_pose2_;
  }
  // Update map to odom tf
  map_br_.sendTransform(tf::StampedTransform(map_to_odom_tf_, ros::Time::now(),
                        "map", "odom"));
}


int main(int argc, char **argv){
  ros::init(argc, argv, "GraphOptimiser");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  GraphOptimiser optimiser(nh, nh_);
  ros::spin();
  return 0;
}
