// created: 13.08.2018
// author: Dario Mammolo

#ifndef GRAPH_OPTIMISATION_H
#define GRAPH_OPTIMISATION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>


class GraphOptimiser{
public:
  GraphOptimiser(ros::NodeHandle nh);
  ~GraphOptimiser();
  void initParams();
  geometry_msgs::PoseStamped createPoseStamped(gtsam::Pose2 pose2);
  void scanMatcherCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg);

private:
  ros::NodeHandle nh_;
  double node_dist_linear_;
  double node_dist_angular_;
  double dist_linear_sq_;
  bool first_scan_pose_;
  int node_counter_;
  gtsam::Pose2 prev_pose2_;
  gtsam::Pose2 current_pose2_;
  nav_msgs::Path graph_path_;
  ros::Subscriber pose_sub_;
  ros::Publisher path_pub_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::noiseModel::Diagonal::shared_ptr scan_match_noise_;
  gtsam::Values pose_estimates_;
};

#endif  // GRAPH_OPTIMISATION_H
