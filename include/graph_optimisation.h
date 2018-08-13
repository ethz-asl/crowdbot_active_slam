// created: 13.08.2018
// author: Dario Mammolo

#ifndef GRAPH_OPTIMISATION_H
#define GRAPH_OPTIMISATION_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

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
  void scanMatcherCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg);

private:
  ros::NodeHandle nh_;
  double node_dist_linear_;
  double node_dist_angular_;
  double dist_linear_sq_;
  bool first_scan_pose_;
  int node_counter_;
  geometry_msgs::Pose2D prev_pose_;
  geometry_msgs::Pose2D current_pose_;
  ros::Subscriber pose_sub;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::noiseModel::Diagonal::shared_ptr scan_match_noise_;
  gtsam::Values pose_estimates_;
};

#endif  // GRAPH_OPTIMISATION_H
