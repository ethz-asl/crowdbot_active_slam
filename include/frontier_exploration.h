#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <queue>

#include <crowdbot_active_slam/get_frontier_list.h>


class FrontierExploration {
public:
  /**
   *  Class Constructor
   */
  FrontierExploration(ros::NodeHandle nh, ros::NodeHandle nh_);

  /**
   *  Class Destructor
   */
  ~FrontierExploration();

  /**
   *  This service callback calculates the frontier centroid informations in
   *  the actual map, returns the centroid information and publishes a cell grid
   *  with the centroids.
   */
  bool serviceCallback(
                  crowdbot_active_slam::get_frontier_list::Request &request,
                  crowdbot_active_slam::get_frontier_list::Response &response);

  /**
   *  Takes the inital cell id and a frontier flag to search for the frontier
   *  region around the initial cell. Returns a Pose2D centroid information.
   *  If the frontier size is too small it returns all values as zero.
   */
  geometry_msgs::Pose2D getFrontierCentroid(unsigned int initial_cell,
                                            std::vector<bool>& frontier_flag,
                                            unsigned int width,
                                            unsigned int height,
                                            float resolution);

  /**
   *  Transforms a cell id to x, y world coordinates.
   */
  void idToWorldXY(unsigned int id, double& x, double& y, unsigned int width,
                   unsigned int height, float resolution);

  /**
   *  Returns a list of cell id's for the 8-neighbourhood around an id cell.
   */
  std::vector<unsigned int> neighbour8(unsigned int id, unsigned int width,
                                       unsigned int height);

  /**
   *  Returns a list of cell id's for the 4-neighbourhood around an id cell.
   */
  std::vector<unsigned int> neighbour4(unsigned int id, unsigned int width,
                                       unsigned int height);

  /**
   *  This callback saves the current occupancy grid map.
   */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

private:
  // Rosparam
  int frontier_size_;

  // Node handler
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Service, Subscriber, Publisher
  ros::ServiceServer service_;
  ros::Subscriber map_sub_;
  ros::Publisher frontier_cell_pub_;

  // Ros msgs and srv
  nav_msgs::OccupancyGrid latest_map_msg_;

  // Tf
  tf::TransformListener robot_pose_listener_;

  // others
  bool map_initialized_;
};

#endif // FRONTIER_EXPLORATION_H
