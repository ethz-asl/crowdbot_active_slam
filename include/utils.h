/**
 *  Adapted from:
 *  https://github.com/ethz-asl/deep_interaction_modeling/blob/master/sf_model/
 *  gazebo_sim/include/utils.hpp
 */

#pragma once
#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <deque>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include <ros/ros.h>

namespace utils {
std::string ParseXacro(const char* cmd);
void PedestrianStartingPose(double limit_x, double limit_y,
                            std::vector<double>* x, std::vector<double>* y,
                            grid_map::GridMap* map);
bool CheckOccupied(std::vector<double>* x, std::vector<double>* y, double x_c,
                   double y_c);
bool CheckIfObstacle(grid_map::Position position, grid_map::GridMap* map);
double RandomFloat(double N);
double CalcMod(double x, double y);
geometry_msgs::Vector3 CalcDirection(geometry_msgs::Vector3 actual,
                                     geometry_msgs::Vector3 desired);
geometry_msgs::Vector3 RotateOfAngle(double x, double y, double theta);
}  // namespace utils
#endif  // INCLUDE_UTILS_H_
