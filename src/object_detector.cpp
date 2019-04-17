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

void ObjectDetector::detectObjectsFromScan(const sensor_msgs::LaserScan& dynamic_laser_msg,
                                           const sensor_msgs::LaserScan& raw_laser_msg,
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
  const double delta_theta = upsampling_factor * dynamic_laser_msg.angle_increment;
  const double constant_factor = sin(delta_theta) / sin(lambda_ - delta_theta);
  const double three_sigma = 3 * sigma_;
  int scan_size = dynamic_laser_msg.ranges.size();
  int jump = 1; // Amount of points to skip when checking distance to the next cluster candidate
  int start_id = 0;

  // Sanity check
  if (scan_size != raw_laser_msg.ranges.size()){
    ROS_WARN("ObjectDetector: dynamic_laser_msg and raw_laser_msg different size!");
    return;
  }

  // Determine first id, which has nonzero values, to find first candidate cluster point
  for (size_t i = 0; i < scan_size; i++){
    if (raw_laser_msg.ranges[i] != 0){
      start_id = i;
      break;
    }
  }

  // Iterate through all scan points and decide size of cluster and if occluded
  for (size_t i = start_id; i < scan_size; i++){
    int next_id = (i + jump) % scan_size;
    // Check if all the scan data has no return
    if (jump == scan_size){
      ROS_WARN("ObjectDetector: Laser scan data has no returns!");
      return;
    }
    // Skip points with no return when considering cluster limits.
    // Most points with no return are in general caused during upsampling
    // in the scan combination for pepper
    else if (raw_laser_msg.ranges[next_id] == 0){
      jump += 1;
      i -= 1;
      continue;
    }

    // Case 1: Current and next scan point are static
    // We only check if the current id corresponds to the start_id. If it does,
    // we finish the first cluster as an empty cluster. It will not be added to
    // any list.
    // If we are at the end we do not need to do anything additonal, as the first
    // cluster is empty and the previous cluster has been added already to a list
    if (dynamic_laser_msg.ranges[next_id] == 0 && dynamic_laser_msg.ranges[i] == 0){
      if (!first_cluster_finished){
        if (i == start_id){
          first_cluster_finished = true;
        }
      }
    }
    // Case 2: Current scan point is dynamic and next scan point is static
    // The main task is here to finish the current cluster built so far.
    // We first check if the first cluster has been finished already, if not,
    // we add the current scan point to the cluster and finish the cluster.
    // If the first cluster has been finished, we finish the temporary cluster,
    // if it is not empty. As example, single point clusters would be detected here
    // as empty clusters (as they have not been added until now). Single point
    // clusters can be noise and are ignored for this reason.
    // When a cluster is finished we check the occlusion.
    // If we are at the end of the loop, we do not need to do anything as the
    // first_cluster is empty (next_id is static).
    else if (dynamic_laser_msg.ranges[next_id] == 0 && dynamic_laser_msg.ranges[i] != 0){
      if (!first_cluster_finished){
        first_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
        first_cluster_finished = true;
        // Get occlusion
        if (raw_laser_msg.ranges[next_id] > dynamic_laser_msg.ranges[i] + 0.1){
          first_cluster_occluded = false;
          start_occluded = true;
        }
        else {
          first_cluster_occluded = true;
          start_occluded = false;
        }
      }
      else {
        if (!temp_cluster.empty()) {
          temp_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
          // Get occlusion
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
          // clear temp_cluster
          temp_cluster.clear();
        }
        // Single point -> ignore
        else {
          // Do nothing
        }
      }
    }
    // Case 3: Current scan point is static and next scan point is dynamic
    // The main task here is to start a new cluster. We check if the new cluster
    // starts occluded or not.
    // If we are at the end of the loop, we add the first_cluster to a list if
    // the size is bigger than one.
    else if (dynamic_laser_msg.ranges[next_id] != 0 && dynamic_laser_msg.ranges[i] == 0){
      // Check if start is occluded or not
      if (dynamic_laser_msg.ranges[next_id] + 0.1 >
          raw_laser_msg.ranges[i]){
        start_occluded = true;
      }
      else {
        start_occluded = false;
      }

      // If we are at the end insert first cluster to corresponding list
      if (i <= scan_size - 1 && i + jump >= scan_size){
        if (first_cluster.size() > 1){
          // Add to corresponding list
          if (start_occluded || first_cluster_occluded){
            occluded_clusters.push_back(first_cluster);
          }
          else {
            normal_clusters.push_back(first_cluster);
          }
        }
      }
    }
    // Case 4: Current and next scan point are dynamic.
    // The main task here is to procceed the clustering. By using de adaptive
    // breakpoint detector, we check if the current and next scan point correspond
    // to the same cluster. We always add the current point to the cluster and
    // only finish the cluster if the detector detects it. We always first check
    // if the first_cluster is already finished and first procceed with this
    // cluster if this one is not finished.
    // When finishing a cluster, we also check the size of the cluster for the
    // reason as in Case 2.
    // If we are at the end of the loop, we merge the temporary cluster and the
    // first_cluster if they correspond to the same cluster, otherwise we add
    // them separately to the corresponding lists.
    else {
      // Compute maximal distance to be in a cluster
      double d_max = dynamic_laser_msg.ranges[i] * constant_factor + three_sigma;

      // Compute distance between two scan points (formula in polar coordinates)
      double distance;
      distance = sqrt(pow(dynamic_laser_msg.ranges[next_id], 2) + pow(dynamic_laser_msg.ranges[i], 2)
            - 2 * dynamic_laser_msg.ranges[next_id] * dynamic_laser_msg.ranges[i] * cos(delta_theta));

      if (distance < d_max){
        if (!first_cluster_finished){
          first_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
        }
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
      else {
        if (!first_cluster_finished){
          first_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
          first_cluster_finished = true;
          // Get occlusion
          if (dynamic_laser_msg.ranges[next_id] + 0.1 >
              dynamic_laser_msg.ranges[i]){
            first_cluster_occluded = false;
          }
          else {
            first_cluster_occluded = true;
          }
          start_occluded = !first_cluster_occluded;
        }
        else {
          if (!temp_cluster.empty()){
            temp_cluster.insert(make_pair(i, dynamic_laser_msg.ranges[i]));
            // Get occlusion
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
            // clear temp_cluster and init start_occluded for next cluster
            temp_cluster.clear();
            start_occluded = !end_occluded;
          }
          // Noise (or single occluded object)
          else {
            //  Get occlusion
            if (dynamic_laser_msg.ranges[next_id] + 0.1 >
                dynamic_laser_msg.ranges[i]){
              start_occluded = true;
            }
            else {
              start_occluded = false;
            }
          }

          // If we are at the end insert first cluster to corresponding list
          if (i <= scan_size - 1 && i + jump >= scan_size){
            if (first_cluster.size() > 1){
              // Add to corresponding list
              if (start_occluded || first_cluster_occluded){
                occluded_clusters.push_back(first_cluster);
              }
              else {
                normal_clusters.push_back(first_cluster);
              }
            }
          }
        }
      }
    }

    // Reset jump variable and update current id for next iteration
    i += jump - 1;
    jump = 1;
  }
}
