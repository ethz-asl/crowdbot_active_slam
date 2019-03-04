/**
 *  object_detector.cpp
 *
 *  Created on: 06.12.2018
 *      Author: Dario Mammolo
 */

#include <object_detector.h>

ObjectDetector::ObjectDetector()
{
  lambda_ = M_PI / 180 * 20;
  sigma_ = 0.01;
}

ObjectDetector::ObjectDetector(double lambda, double sigma):
  lambda_(M_PI / 180 * lambda), sigma_(sigma)
{

}

ObjectDetector::~ObjectDetector(){}

void ObjectDetector::detectObjectsFromScan(sensor_msgs::LaserScan& dynamic_laser_msg,
                                           sensor_msgs::LaserScan& raw_laser_msg,
                                           list<map<int, double>>& normal_clusters,
                                           list<map<int, double>>& occluded_clusters,
                                           int upsampling_factor){
  // Init cluster maps
  map<int, double> temp_cluster;
  map<int, double> first_cluster;

  // Init booleans
  bool start_occluded = false;
  bool end_occluded = false;
  bool first_cluster_occluded = false;
  bool first_cluster_finished = false;

  // Predefine constant variables
  double delta_theta = upsampling_factor * dynamic_laser_msg.angle_increment;
  double constant_factor = sin(delta_theta) / sin(lambda_ - delta_theta);
  double three_sigma = 3 * sigma_;
  int scan_size = dynamic_laser_msg.ranges.size();
  int jump = 1;
  int start_id = 0;

  // Determine first id which has nonzero values
  for (size_t i = 0; i < scan_size; i++){
    if (raw_laser_msg.ranges[i] != 0){
      start_id = i;
      break;
    }
  }

  // Iterate through all scan points and decide size of cluster and if occluded
  for (size_t i = start_id; i < scan_size; i++){
    if (raw_laser_msg.ranges[(i + jump) % scan_size] == 0){
      jump += 1;
      i -= 1;
      continue;
    }
    int next_id = (i + jump) % scan_size;
    if ((i <= scan_size - 1 && i + jump >= scan_size &&
         dynamic_laser_msg.ranges[next_id] != 0)
        || (i != scan_size - 1 && dynamic_laser_msg.ranges[next_id] != 0)){
      if (dynamic_laser_msg.ranges[i] != 0){
        // Compute maximal distance to be in a cluster
        double d_max = dynamic_laser_msg.ranges[i] * constant_factor + three_sigma;

        // Compute distance between two scan points (formula in polar coordinates)
        double distance;
        distance = sqrt(pow(dynamic_laser_msg.ranges[next_id], 2) + pow(dynamic_laser_msg.ranges[i], 2)
              - 2 * dynamic_laser_msg.ranges[next_id] * dynamic_laser_msg.ranges[i] * cos(delta_theta));

        // Check if distance smaller than d_max
        if (distance < d_max){
          // Check if first cluster already finished building
          if (!first_cluster_finished){
            first_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
          }
          // If finished building first cluster
          else {
            temp_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
            // If we are at the last scan point merge temp_cluster with first_cluster
            if (i <= scan_size - 1 && i + jump >= scan_size){
              if (!first_cluster.empty()){
                temp_cluster.insert(first_cluster.begin(), first_cluster.end());
              }
              // Add to corresponding list
              if (start_occluded || first_cluster_occluded){
                occluded_clusters.push_back(temp_cluster);
              }
              else {
                normal_clusters.push_back(temp_cluster);
              }
            }
          }
        }
        // If distance is bigger finish current cluster
        else {
          // Check if first cluster already finished
          if (!first_cluster_finished){
            first_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
            first_cluster_finished = true;
            if (dynamic_laser_msg.ranges[next_id] + 0.1 >
                dynamic_laser_msg.ranges[i]){
              first_cluster_occluded = false;
              start_occluded = true;
            }
            else {
              first_cluster_occluded = true;
              start_occluded = false;
            }
          }
          else {
            // Check if temp_cluster not empty
            if (!temp_cluster.empty()) {
              temp_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));

              // Check if end is occluded or not
              if (dynamic_laser_msg.ranges[next_id] + 0.1 >
                  dynamic_laser_msg.ranges[i]){
                end_occluded = false;
              }
              else {
                end_occluded = true;
              }

              // Add to corresponding list
              if (start_occluded || end_occluded){
                occluded_clusters.push_back(temp_cluster);
              }
              else {
                normal_clusters.push_back(temp_cluster);
              }
              // clear temp_cluster and init start_occluded bool
              temp_cluster.clear();
              start_occluded = !end_occluded;
            }
            // Noise (or single occluded object)
            else {
              // Check if start is occluded or not
              if (dynamic_laser_msg.ranges[next_id] + 0.1 >
                  dynamic_laser_msg.ranges[i]){
                start_occluded = true;
              }
              else {
                start_occluded = false;
              }
            }
          }
        }
      }
      else{
        // Check if start is occluded or not
        if (dynamic_laser_msg.ranges[next_id] + 0.1 >
            raw_laser_msg.ranges[i]){
          start_occluded = true;
        }
        else {
          start_occluded = false;
        }
      }
    }
    // If the range is zero, means scan has been assigned as static
    // close cluster and check if occluded or not
    else {
      // Finish current cluster
      // Check if first cluster already finished
      if (!first_cluster_finished){
        if (i == start_id && dynamic_laser_msg.ranges[start_id] == 0){
          first_cluster_finished = true;
        }
        else {
          first_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
          first_cluster_finished = true;
          if (raw_laser_msg.ranges[next_id] > dynamic_laser_msg.ranges[i] + 0.1){
            first_cluster_occluded = false;
            start_occluded = true;
          }
          else {
            first_cluster_occluded = true;
            start_occluded = false;
          }
        }
      }
      else {
        // Check if temp_cluster not empty
        if (!temp_cluster.empty()) {
          temp_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));

          // Check if end is occluded or not
          if (raw_laser_msg.ranges[next_id] > dynamic_laser_msg.ranges[i] + 0.1){
            end_occluded = false;
          }
          else {
            end_occluded = true;
          }

          // Add to corresponding list
          if (start_occluded || end_occluded){
            occluded_clusters.push_back(temp_cluster);
          }
          else {
            normal_clusters.push_back(temp_cluster);
          }
          // clear temp_cluster and init start_occluded bool
          temp_cluster.clear();
          start_occluded = !end_occluded;
        }
        // Means actual range and next range are both assigned as static
        else {
          // Do nothing
        }
      }
    }
    // Reset jump
    i += jump - 1;
    jump = 1;
  }
}
