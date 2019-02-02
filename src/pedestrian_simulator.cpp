/**
 *  Adapted from:
 *  https://github.com/ethz-asl/deep_interaction_modeling/blob/master/sf_model/
 *  gazebo_sim/src/social_force_model.cpp
 */

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>  // for cosine
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <deque>
#include <fstream>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <limits>
#include <pedestrian.h>
#include <sstream>
#include <string>
#include <utils.h>
#include <vector>

#define PI 3.14159265

using namespace grid_map;


/**
 *  A helper function which creates map ids from index.
 */
int mapIndexToId(int x, int y, int width){
  int id = x + y * width;
  return id;
}

// Functions from SDF calculations
//--------------------------------------------------------------------------

/**
 *  A helper function which creates map index from position information.
 */
std::vector<int> positionToMapIndex(double x, double y,
                                    int width, int height, float resolution){
  std::vector<int> index(2);
  index[0] = floor(x / resolution) + width / 2;
  index[1] = floor(y / resolution) + height / 2;

  return index;
}

void findChange(bool& change, int occupation, int current_occupation){
  if (current_occupation != occupation){
    if (std::abs(current_occupation + occupation) == 1){
      change = true;
    }
  }
}

double xCircleMinDist(int ix, int iy, int n_cells, Eigen::MatrixXi& map,
                      int& x_closest, int& y_closest){
  // https://www.geeksforgeeks.org/bresenhams-circle-drawing-algorithm/
  float dist_sq = -1;
  int occupation = map(ix, iy);
  int x = n_cells;
  int y = 0;
  int err = 3 - (n_cells << 1);
  bool found_change = false;

  int xc, yc;
  if (occupation != 0){
    xc = x - 1;
    yc = y;
  }
  else {
    xc = x;
    yc = y;
  }

  while (x >= y){
    findChange(found_change, occupation, map(ix + x, iy + y)); // 1. octant
    if (found_change){
      double dist_sq_temp = xc * xc + yc * yc;
      if (dist_sq == -1 || dist_sq > dist_sq_temp){
        dist_sq = dist_sq_temp;
        x_closest = ix + x;
        y_closest = iy + y;
      }
    }

    findChange(found_change, occupation, map(ix + y, iy + x)); // 2. octant
    if (found_change){
      double dist_sq_temp = xc * xc + yc * yc;
      if (dist_sq == -1 || dist_sq > dist_sq_temp){
        dist_sq = dist_sq_temp;
        x_closest = ix + y;
        y_closest = iy + x;
      }
    }

    findChange(found_change, occupation, map(ix - y, iy + x)); // 3. octant
    if (found_change){
      double dist_sq_temp = xc * xc + yc * yc;
      if (dist_sq == -1 || dist_sq > dist_sq_temp){
        dist_sq = dist_sq_temp;
        x_closest = ix - y;
        y_closest = iy + x;
      }
    }

    findChange(found_change, occupation, map(ix - x, iy + y)); // 4. octant
    if (found_change){
      double dist_sq_temp = xc * xc + yc * yc;
      if (dist_sq == -1 || dist_sq > dist_sq_temp){
        dist_sq = dist_sq_temp;
        x_closest = ix - x;
        y_closest = iy + y;
      }
    }

    findChange(found_change, occupation, map(ix - x, iy - y)); // 5. octant
    if (found_change){
      double dist_sq_temp = xc * xc + yc * yc;
      if (dist_sq == -1 || dist_sq > dist_sq_temp){
        dist_sq = dist_sq_temp;
        x_closest = ix - x;
        y_closest = iy - y;
      }
    }

    findChange(found_change, occupation, map(ix - y, iy - x)); // 6. octant
    if (found_change){
      double dist_sq_temp = xc * xc + yc * yc;
      if (dist_sq == -1 || dist_sq > dist_sq_temp){
        dist_sq = dist_sq_temp;
        x_closest = ix - y;
        y_closest = iy - x;
      }
    }

    findChange(found_change, occupation, map(ix + y, iy - x)); // 7. octant
    if (found_change){
      double dist_sq_temp = xc * xc + yc * yc;
      if (dist_sq == -1 || dist_sq > dist_sq_temp){
        dist_sq = dist_sq_temp;
        x_closest = ix + y;
        y_closest = iy - x;
      }
    }

    findChange(found_change, occupation, map(ix + x, iy - y)); // 8. octant
    if (found_change){
      double dist_sq_temp = xc * xc + yc * yc;
      if (dist_sq == -1 || dist_sq > dist_sq_temp){
        dist_sq = dist_sq_temp;
        x_closest = ix + x;
        y_closest = iy - y;
      }
    }

    y++;
    if (err > 0){
      x--;
      err += ((y - x) << 2) + 10; //<< 2 -> 4*
    }
    else {
      err += (y << 2) + 6;
    }

    if (occupation != 0){
      yc = y - 1;
      xc = x - 1;
    }
    else {
      yc = y;
      xc = x;
    }
    found_change = false;
  }
  if (dist_sq != -1) return sqrt(dist_sq);
  else return dist_sq;
}
//-------------------------------------------------------------------------

geometry_msgs::Vector3 ForceToDestination(Pedestrian* ped, double desired_speed,
                                          double rel_time, double tol, int N,
                                          double* angular,
                                          grid_map::GridMap* map) {
  geometry_msgs::Vector3 e_t;  // Just the direction along which apply the speed
  geometry_msgs::Vector3 force_dest;
  e_t = utils::CalcDirection(ped->GetCurrentPose(), ped->GetGoal());

  double current_yaw = tf::getYaw(ped->GetOrientation());
  double error = 0.0 - current_yaw;
  // PID for fixed pedestrian angular position
  if (!std::isfinite(error)) {
    error = 0;
  }  // This is necessary cause at the first iterations we get weird values
  if (abs(error) > PI) {
    error = 2 * PI - abs(error);
  }
  *angular = error;
  // Calculate Force
  force_dest.x = (e_t.x * desired_speed - ped->GetSpeed().x) / rel_time;
  force_dest.y = (e_t.y * desired_speed - ped->GetSpeed().y) / rel_time;

  // Check for stuck pedestrian (almost notmoving anymore)
  if (abs(ped->GetSpeed().x) + abs(ped->GetSpeed().y) <= 0.05){
    ped->PubGoal(map);
  }

  // If we are almost there set a new desired pose
  if (abs(ped->GetCurrentPose().x - ped->GetGoal().x) < tol &&
      abs(ped->GetCurrentPose().y - ped->GetGoal().y) < tol) {
    ped->PubGoal(map);
  }
  return force_dest;
}

geometry_msgs::Vector3 RepulsiveWallForce(Pedestrian* ped_i,
                                         grid_map::GridMap* map,
                                         double delta_t) {
  double A = 4.3, B = 1.07, l = 0.7, g_0;
  geometry_msgs::Vector3 force, closest_obj, rel_dist, speed, step, r_s;
  Position ped_position;
  ped_position.x() = ped_i->GetCurrentPose().x;
  ped_position.y() = ped_i->GetCurrentPose().y;
  closest_obj.x = map->atPosition("Closest X", ped_position);
  closest_obj.y = map->atPosition("Closest Y", ped_position);

  rel_dist.x = ped_position.x() - closest_obj.x;
  rel_dist.y = ped_position.y() - closest_obj.y;

  speed = ped_i->GetSpeed();
  step.x = -speed.x * delta_t;
  step.y = -speed.y * delta_t;

  r_s.x = rel_dist.x - step.x;
  r_s.y = rel_dist.y - step.y;

  double mod_rd = utils::CalcMod(rel_dist.x, rel_dist.y);
  double mod_step = utils::CalcMod(step.x, step.y);
  double mod_r_s = utils::CalcMod(r_s.x, r_s.y);

  double b = sqrt(pow(mod_rd + mod_r_s, 2) - pow(mod_step, 2)) / 2;
  if (mod_rd != 0 && mod_r_s != 0 && mod_step != 0 && b != 0) {
    g_0 = A * exp(-b / B) * (mod_rd + mod_r_s) / (2 * b);

    force.x = g_0 * (rel_dist.x / mod_rd + r_s.x / mod_r_s) / 2;
    force.y = g_0 * (rel_dist.y / mod_rd + r_s.y / mod_r_s) / 2;
  } else {
    force.x = 0;
    force.y = 0;
  }
  return force;
}

geometry_msgs::Vector3 RepulsivePedForce(Pedestrian* ped_i, Pedestrian* ped_j,
                                         double delta_t) {
  // Parameters
  double A = 4.3;
  double B = 1.07;
  double l = 0.7;

  double b = 0;
  double g_0, w;
  double mod_r_ij, mod_r_s_ij, mod_s_ij;
  double cos_phi;
  geometry_msgs::Vector3 r_ij;
  geometry_msgs::Vector3 r_s_ij, s_ij;
  geometry_msgs::Vector3 g;
  geometry_msgs::Vector3 force;
  geometry_msgs::Vector3 e_t_i;  // The direction along which apply the speed

  r_ij.x = ped_i->GetCurrentPose().x - ped_j->GetCurrentPose().x;
  r_ij.y = ped_i->GetCurrentPose().y - ped_j->GetCurrentPose().y;

  s_ij.x = (ped_j->GetSpeed().x - ped_i->GetSpeed().x) * delta_t;
  s_ij.y = (ped_j->GetSpeed().y - ped_i->GetSpeed().y) * delta_t;

  r_s_ij.x = r_ij.x - s_ij.x;
  r_s_ij.y = r_ij.y - s_ij.y;

  mod_r_ij = utils::CalcMod(r_ij.x, r_ij.y);
  mod_r_s_ij = utils::CalcMod(r_s_ij.x, r_s_ij.y);
  mod_s_ij = utils::CalcMod(s_ij.x, s_ij.y);

  b = sqrt(pow(mod_r_ij + mod_r_s_ij, 2) - pow(mod_s_ij, 2)) / 2;
  // If necessary because at the beginning of the sim we get weird values
  if (mod_r_ij != 0 && mod_r_s_ij != 0 && mod_s_ij != 0 && b != 0) {
    g_0 = A * exp(-b / B) * (mod_r_ij + mod_r_s_ij) / (2 * b);

    g.x = g_0 * (r_ij.x / mod_r_ij + r_s_ij.x / mod_r_s_ij) / 2;
    g.y = g_0 * (r_ij.y / mod_r_ij + r_s_ij.y / mod_r_s_ij) / 2;

    e_t_i = utils::CalcDirection(ped_i->GetCurrentPose(), ped_i->GetGoal());
    // Get the cos with the scalar product between pedestrian orientation and
    // relative position of other ped
    cos_phi = (e_t_i.x * r_ij.x / mod_r_ij) + (e_t_i.y * r_ij.y / mod_r_ij);
    w = l + (1 - l) * (1 + cos_phi) / 2;
    // Finally get the force
    force.x = w * g.x;
    force.y = w * g.y;
  } else {
    force.x = 0;
    force.y = 0;
  }
  return force;
}

geometry_msgs::Vector3 RepulsivePioneerForce(Pedestrian* ped_i,
                                  geometry_msgs::Vector3& pioneer_position,
                                  geometry_msgs::Vector3& pioneer_speed,
                                  double delta_t) {
  // Parameters
  double A = 6.3;
  double B = 1.07;
  double l = 0.7;

  double b = 0;
  double g_0, w;
  double mod_r_ij, mod_r_s_ij, mod_s_ij;
  double cos_phi;
  geometry_msgs::Vector3 r_ij;
  geometry_msgs::Vector3 r_s_ij, s_ij;
  geometry_msgs::Vector3 g;
  geometry_msgs::Vector3 force;
  geometry_msgs::Vector3 e_t_i;  // The direction along which apply the speed

  r_ij.x = ped_i->GetCurrentPose().x - pioneer_position.x;
  r_ij.y = ped_i->GetCurrentPose().y - pioneer_position.y;

  s_ij.x = (pioneer_speed.x - ped_i->GetSpeed().x) * delta_t;
  s_ij.y = (pioneer_speed.y - ped_i->GetSpeed().y) * delta_t;

  r_s_ij.x = r_ij.x - s_ij.x;
  r_s_ij.y = r_ij.y - s_ij.y;

  mod_r_ij = utils::CalcMod(r_ij.x, r_ij.y);
  mod_r_s_ij = utils::CalcMod(r_s_ij.x, r_s_ij.y);
  mod_s_ij = utils::CalcMod(s_ij.x, s_ij.y);

  b = sqrt(pow(mod_r_ij + mod_r_s_ij, 2) - pow(mod_s_ij, 2)) / 2;
  // If necessary because at the beginning of the sim we get weird values
  if (mod_r_ij != 0 && mod_r_s_ij != 0 && mod_s_ij != 0 && b != 0) {
    g_0 = A * exp(-b / B) * (mod_r_ij + mod_r_s_ij) / (2 * b);

    g.x = g_0 * (r_ij.x / mod_r_ij + r_s_ij.x / mod_r_s_ij) / 2;
    g.y = g_0 * (r_ij.y / mod_r_ij + r_s_ij.y / mod_r_s_ij) / 2;

    e_t_i = utils::CalcDirection(ped_i->GetCurrentPose(), ped_i->GetGoal());
    // Get the cos with the scalar product between pedestrian orientation and
    // relative position of other ped
    cos_phi = (e_t_i.x * r_ij.x / mod_r_ij) + (e_t_i.y * r_ij.y / mod_r_ij);
    w = l + (1 - l) * (1 + cos_phi) / 2;
    // Finally get the force
    force.x = w * g.x;
    force.y = w * g.y;
  } else {
    force.x = 0;
    force.y = 0;
  }
  return force;
}

int main(int argc, char** argv) {
  int N;
  int robot_not_seen;
  double dt;
  double tolerance;
  double desired_speed;
  double rel_time;
  double delta_t;
  double map_size_x, map_size_y, map_center_x, map_center_y, map_res;
  std::string base_name;
  std::string occupancy_grid_map_file;
  std::string sdf_map_file;

  srand(time(NULL));
  // Start node and service client
  ros::init(argc, argv, "spawner_node");
  ros::NodeHandle nh("~");
  ros::ServiceClient gazebo_spawn_clt =
      nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  ros::ServiceClient gazebo_delete_clt =
      nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  ros::ServiceClient pause_simulation =
      nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  ros::ServiceClient unpause_simulation =
      nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  ros::ServiceClient pioneer_state =
      nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::service::waitForService("/gazebo/spawn_urdf_model");
  ros::Publisher map_publisher =
      nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  nh.param<int>("n_ped", N, 30);
  nh.param<std::string>("/ped_name", base_name, "pedestrian");
  nh.param<std::string>("/occupancy_map_file", occupancy_grid_map_file,
                        "occupancy_grid_map_lab_2000x2000.txt");
  nh.param<double>("/dt", dt, 0.2);
  nh.param<double>("/map_size_x", map_size_x, 30.0);
  nh.param<double>("/map_size_y", map_size_y, 35.0);
  nh.param<double>("/map_center_x", map_center_x, 9.5);
  nh.param<double>("/map_center_y", map_center_y, 4.0);
  nh.param<double>("/map_resolution", map_res, 0.05);
  nh.param<int>("/robot_not_seen", robot_not_seen, 0);

  // Simulation parameters
  nh.param<double>("/goal_tol", tolerance, 0.5);
  nh.param<double>("/desired_speed", desired_speed, 0.8);
  nh.param<double>("/relaxation_time", rel_time, 0.5);
  nh.param<double>("/delta_t", delta_t, 0.1);

  // Create service message request
  gazebo_msgs::SpawnModel model;
  std_srvs::Empty physics;

  // Pause simulation while spawning objects
  pause_simulation.call(physics);

  ros::Duration interval(dt);

  // Read occupancy grid map and sdf map
  //--------------------------------------------------------------------------
  // Get path to occupancy grid map and sdf map file
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string occupancy_grid_map_path = package_path +
                          "/worlds/occupancy_maps/" + occupancy_grid_map_file;

  int full_map_width = 2000;
  int full_map_height = 2000;
  float full_map_resolution = 0.05;
  Eigen::MatrixXi occ_mat_full(full_map_width, full_map_height);

  std::ifstream occ_map_file(occupancy_grid_map_path.c_str());
  std::string line;
  int id = 0;

  if (occ_map_file.is_open()){
    int x_cell, y_cell;
    while (std::getline(occ_map_file, line)){
      y_cell = id / full_map_width;
      x_cell = id - y_cell * full_map_width;
      std::stringstream ss_ref;
      ss_ref << line;
      double p_ref;
      ss_ref >> p_ref;

      if (p_ref == -1){
        occ_mat_full(x_cell, y_cell) = 1;
      }
      else if (p_ref >= 50){
        occ_mat_full(x_cell, y_cell) = 1;
      }
      else {
        occ_mat_full(x_cell, y_cell) = 0;
      }
      id += 1;
    }
  }
  else {
    ROS_INFO("Failed to open occupancy map file!");
  }

  //--------------------------------------------------------------------------

  // Build up the map
  //--------------------------------------------------------------------------
  grid_map::GridMap map({"Occupied", "Closest X", "Closest Y", "Distance"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(map_size_x, map_size_y), map_res,
                  grid_map::Position(map_center_x, map_center_y));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(), map.getSize()(0),
           map.getSize()(1));
  ROS_INFO("The center of the map is located at (%f, %f) in the %s frame.",
           map.getPosition().x(), map.getPosition().y(),
           map.getFrameId().c_str());

  std::vector<int> p_start = positionToMapIndex(map_center_x - map_size_x / 2,
                                               map_center_y - map_size_y / 2,
                                               full_map_width, full_map_height,
                                               full_map_resolution);
  std::vector<int> p_end = positionToMapIndex(map_center_x + map_size_x / 2,
                                             map_center_y + map_size_y / 2,
                                             full_map_width, full_map_height,
                                             full_map_resolution);

  int width = map_size_x / map_res;
  int height = map_size_y / map_res;

  // Fill the map
  // First get interest occ_map
  Eigen::MatrixXi occ_mat(width, height);
  for (int i = p_start[0]; i <= p_end[0]; i++){
    for (int j = p_start[1]; j <= p_end[1]; j++){
      occ_mat(i - p_start[0], j - p_start[1]) = occ_mat_full(i, j);
    }
  }
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    int index = it.getLinearIndex();
    int iy_cell = index / width;
    int ix_cell = index - iy_cell * width;
    map.at("Occupied", *it) = occ_mat(width - ix_cell - 1, height - iy_cell - 1);
  }
  ROS_INFO("Occupancy map filled");


  // Calculate the distances.
  int n = 1;
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    //
    int index = it.getLinearIndex();
    int iy_cell = index / width;
    int ix_cell = index - iy_cell * width;

    int i = width - ix_cell - 1 + p_start[0];
    if (i == p_end[0] - 1) n = 1;

    double dist = -1;
    int x_closest, y_closest;
    bool not_found = true;
    int j = height - iy_cell - 1 + p_start[1];
    // Search for shortest distance of (i, j)
    dist = -1;
    not_found = true;
    while (not_found){
      dist = xCircleMinDist(i, j, n, occ_mat_full, x_closest, y_closest);
      // std::cout << n << std::endl;
      if (dist == -1){
        n += 1;
      }
      else {
        not_found = false;
        int temp_x_closest, temp_y_closest;
        double temp_dist = xCircleMinDist(i, j, n + 1, occ_mat_full,
                                          temp_x_closest, temp_y_closest);
        if (dist > temp_dist && temp_dist != -1){
          dist = temp_dist;
          x_closest = temp_x_closest;
          y_closest = temp_y_closest;
        }
        if (n > 2) n -= 2;
        else n = 1;
      }
    }

    map.at("Closest X", *it) = (x_closest - full_map_width / 2) * map_res;
    map.at("Closest Y", *it) = (y_closest - full_map_height / 2) * map_res;
    map.at("Distance", *it) = dist * map_res;
  }
  ROS_INFO("Map ready");
  grid_map_msgs::GridMap map_msg;
  GridMapRosConverter::toMessage(map, map_msg);
  map_publisher.publish(map_msg);
  //--------------------------------------------------------------------------

  // Create Pedestrian vector
  //--------------------------------------------------------------------------
  std::vector<Pedestrian> pedestrians(N);
  // Calculate positions for the pedestrians
  std::vector<double> x(N), y(N);
  utils::PedestrianStartingPose(map_size_x / 2.1, map_size_y / 2.1, &x, &y,
                                &map);

  // Spawn pedestrians
  for (int i = 0; i < N; i++) {
    model.request.initial_pose.position.x = x.at(i);
    model.request.initial_pose.position.y = y.at(i);
    model.request.initial_pose.position.z = 0;
    tf::Quaternion q = tf::createQuaternionFromYaw(0.0);
    model.request.initial_pose.orientation.x = q.x();
    model.request.initial_pose.orientation.y = q.y();
    model.request.initial_pose.orientation.z = q.z();
    model.request.initial_pose.orientation.w = q.w();
    std::string name(base_name + std::to_string(i));
    std::string robot_namespace(base_name + std::to_string(i) + "_ns");
    model.request.model_name = name;
    model.request.reference_frame = "world";
    model.request.robot_namespace = robot_namespace;
    // Parse urdf model adding the link namespace for tf.
    std::string cmd = "xacro --inorder '" +
                      ros::package::getPath("crowdbot_active_slam") +
                      "/urdf/pedestrian.xacro' prefix:=" + robot_namespace;
    model.request.model_xml = utils::ParseXacro(cmd.c_str());
    gazebo_spawn_clt.call(model);

    // Save model parameters
    pedestrians.at(i).SetName(name);
    pedestrians.at(i).SetNumber(i);
    pedestrians.at(i).SetElapsedTime(ros::Time::now());
    pedestrians.at(i).rosPub_cmd_vel = nh.advertise<geometry_msgs::Twist>(
        "/" + robot_namespace + "/cmd_vel", 1, true);
    pedestrians.at(i).rosPub_goal = nh.advertise<geometry_msgs::Vector3>(
        "/" + robot_namespace + "/ped_goal", 1, true);
    pedestrians.at(i).rosSub_odom =
        nh.subscribe("/" + robot_namespace + "/odom", 1,
                     &Pedestrian::SetCurrentPose, &pedestrians.at(i));
    // The set desired callback is also the one entitled in showing the goal
    // in the simulation.
    // With the latching there is no need to keep on plublishing the goal.
    // We just do it when we update it.
    pedestrians.at(i).PubGoal(&map);
  }
  ROS_INFO("All pedestrians spawned.");
  //--------------------------------------------------------------------------

  // Unpause simulation now
  unpause_simulation.call(physics);

  // Set loop rate
  ros::Rate loop_rate(15);

  // Calculate Forces
  while (ros::ok()) {
    // Get current pioneer position and speed
    gazebo_msgs::GetModelState pioneer_state_msg;
    pioneer_state_msg.request.model_name = "pioneer";
    pioneer_state.call(pioneer_state_msg);
    // Position
    geometry_msgs::Vector3 pioneer_position;
    pioneer_position.x = pioneer_state_msg.response.pose.position.x;
    pioneer_position.y = pioneer_state_msg.response.pose.position.y;
    // Speed rotated into world frame
    tf::Quaternion pioneer_orientation;
    quaternionMsgToTF(pioneer_state_msg.response.pose.orientation,
                      pioneer_orientation);
    double theta = tf::getYaw(pioneer_orientation);

    geometry_msgs::Vector3 pioneer_speed;
    pioneer_speed.x = cos(theta) * pioneer_state_msg.response.twist.linear.x -
                      sin(theta) * pioneer_state_msg.response.twist.linear.y;
    pioneer_speed.y = sin(theta) * pioneer_state_msg.response.twist.linear.x +
                      cos(theta) * pioneer_state_msg.response.twist.linear.y;

    // starting pedestrian index
    int starting_ped_index = 0;

    // Now we apply the force model
    for (int i = starting_ped_index; i < N; i++) {
      geometry_msgs::Vector3 total_force, obj_force;
      geometry_msgs::Twist velocity;
      double angular;  // Used by PID to make the robot stay axis oriented
      // Calculate force given by destination
      total_force = ForceToDestination(&pedestrians.at(i), desired_speed,
                                       rel_time, tolerance, N, &angular, &map);
      // Calculate object repulsive force
      obj_force = RepulsiveWallForce(&pedestrians.at(i), &map, delta_t);
      total_force.x += obj_force.x;
      total_force.y += obj_force.y;
      // Calculate pedestrians repulsive forces
      for (int j = robot_not_seen; j < N; j++) {
        if (j != i) {
          if (abs(pedestrians.at(i).GetCurrentPose().x -
                  pedestrians.at(j).GetCurrentPose().x) < 2.0 &&
              abs(pedestrians.at(i).GetCurrentPose().y -
                  pedestrians.at(j).GetCurrentPose().y) < 2.0){
            geometry_msgs::Vector3 repulsive_force;
            repulsive_force = RepulsivePedForce(&pedestrians.at(i),
                                                &pedestrians.at(j), delta_t);
            total_force.x += repulsive_force.x;
            total_force.y += repulsive_force.y;
          }
        }
      }

      // Calculate pioneer repulsive force
      if (abs(pedestrians.at(i).GetCurrentPose().x - pioneer_position.x) < 4.0 &&
          abs(pedestrians.at(i).GetCurrentPose().y - pioneer_position.y) < 4.0){
        geometry_msgs::Vector3 pioneer_force;
        pioneer_force = RepulsivePioneerForce(&pedestrians.at(i), pioneer_position,
                                              pioneer_speed, delta_t);
        total_force.x += pioneer_force.x;
        total_force.y += pioneer_force.y;
      }

      velocity.linear.x = pedestrians.at(i).GetSpeed().x + dt * total_force.x;
      velocity.linear.y = pedestrians.at(i).GetSpeed().y + dt * total_force.y;

      double mod_speed = utils::CalcMod(velocity.linear.x, velocity.linear.y);
      double max_speed = 1.3 * desired_speed;
      if (mod_speed >= max_speed) {
        velocity.linear.x = velocity.linear.x * max_speed / mod_speed;
        velocity.linear.y = velocity.linear.y * max_speed / mod_speed;
      }
      velocity.angular.z = angular;

      if (std::isfinite(velocity.linear.x) &&
          std::isfinite(velocity.linear.y)) {
        ros::Time time = ros::Time::now();
        if (time - pedestrians.at(i).GetElapsedTime() >= interval) {
          pedestrians.at(i).rosPub_cmd_vel.publish(velocity);
          pedestrians.at(i).SetElapsedTime(time);
        }
      } else {
        ROS_ERROR("NAN or INF at %d. X = %lf, Y = %lf", i, velocity.linear.x,
                  velocity.linear.y);
      }
    }
    // Only when we calculated everything for every pedestrian
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
