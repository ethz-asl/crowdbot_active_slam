/**
 *  static_laser_scan_combiner.cpp
 *
 *  Created on: 07.12.2018
 *      Author: Dario Mammolo
 */

#include <ros/ros.h>
#include <static_laser_scan_combiner.h>

using namespace std;

StaticLaserScanCombiner::StaticLaserScanCombiner
  (ros::NodeHandle nh, ros::NodeHandle nh_):nh_(nh), nh_private_(nh_)
{
  ROS_INFO("Started StaticLaserScanCombiner");
  object_detector_ = ObjectDetector(20, 0.01);
  static_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("static_combined_scan", 1);
  scan_sub_ = nh_.subscribe("/base_scan", 1, &StaticLaserScanCombiner::scanCallback, this);
  marker_front_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_front", 1);
  marker_occluded_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_occluded", 1);

  begin_time_ = ros::Time::now();
  // Wait until time is not 0
  while (begin_time_.toSec() == 0) begin_time_ = ros::Time::now();
  initialized_first_scan_ = false;
}

StaticLaserScanCombiner::~StaticLaserScanCombiner(){}

void StaticLaserScanCombiner::initScan(sensor_msgs::LaserScan& laser_msg,
                                        sensor_msgs::LaserScan& init_scan){
  // Sum and sort scans
  size_t laser_scan_size = laser_msg_.ranges.size();
  for (size_t i = 0; i < laser_scan_size; i++){
    init_scan_sum_[i].push_back(laser_msg.ranges[i]);
    sort(init_scan_sum_[i].begin(), init_scan_sum_[i].end());
  }

  // If passed time is higher than TODO: define var
  ros::Time end = ros::Time::now();
  double diff = (end - begin_time_).toSec();
  if (diff > 4){
    int count_unknown = 0;
    double temp_median;
    size_t scan_sum_size = init_scan_sum_[0].size();
    for (size_t j = 0; j < laser_scan_size; j++){
      // Calculate Medians
      if (scan_sum_size % 2 == 0){
        temp_median = (init_scan_sum_[j][(scan_sum_size / 2) - 1] +
                       init_scan_sum_[j][(scan_sum_size / 2)]) / 2;
      }
      else {
        temp_median = init_scan_sum_[j][(scan_sum_size - 1) / 2];
      }

      /// Calculate means
      // Init count and sum var for mean computation
      int count = 0;
      double sum = 0;
      // TODO: var for 0.02
      for (size_t k = 0; k < scan_sum_size; k++){
        if (abs(temp_median - init_scan_sum_[j][k]) < 0.02){
          sum += init_scan_sum_[j][k];
          count += 1;
        }
      }
      // Compute mean if more than 50% of all values are in range
      if (count > scan_sum_size * 0.5){
        init_scan.ranges[j] = sum / double(count);
      }
      else {
        init_scan.ranges[j] = 0;
        count_unknown += 1;
      }
    }

    // Generate init scan if enogh scan points available, TODO: var for 50
    if (count_unknown < 50){
      // Publish twice to initialize the map in graph_optimisation
      static_scan_pub_.publish(init_scan);
      ros::Duration(0.5).sleep();
      static_scan_pub_.publish(init_scan);
      initialized_first_scan_ = true;
    }
  }
}

void StaticLaserScanCombiner::scanCallback
     (const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  laser_msg_ = *scan_msg;
  size_t laser_scan_size = laser_msg_.ranges.size();

  if (init_scan_sum_.empty()) init_scan_sum_.resize(laser_scan_size);
  if (!initialized_first_scan_) {
    sensor_msgs::LaserScan init_scan = laser_msg_;
    initScan(laser_msg_, init_scan);
  }
  else {
    list<map<int, double>> normal_clusters;
    list<map<int, double>> occluded_clusters;
    object_detector_.detectObjectsFromScan(laser_msg_, normal_clusters,
                                                       occluded_clusters);

    visualization_msgs::Marker line_list;
    line_list.header.frame_id = laser_msg_.header.frame_id;
    line_list.header.stamp = laser_msg_.header.stamp;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.1;
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    sensor_msgs::LaserScan occluded_scan = laser_msg_;

    for (auto it = normal_clusters.begin(); it != normal_clusters.end(); ++it){
      geometry_msgs::Point line_start,line_end;
      for (auto map_it = (*it).begin(); map_it != (*it).end(); ++map_it){
        if (map_it->first < 720){
          occluded_scan.ranges[map_it->first] = 0;
        }
        else {
          occluded_scan.ranges[map_it->first - 720] = 0;

        }
        if (map_it == (*it).begin()){
          line_start.x = map_it->second * cos(map_it->first * laser_msg_.angle_increment+M_PI);
          line_start.y = map_it->second * sin(map_it->first * laser_msg_.angle_increment+M_PI);
          line_start.z = 0;
        }
        else if (map_it == --(*it).end()){
          line_end.x = map_it->second * cos(map_it->first * laser_msg_.angle_increment+M_PI);
          line_end.y = map_it->second * sin(map_it->first * laser_msg_.angle_increment+M_PI);
          line_end.z = 0;
        }
      }
      line_list.points.push_back(line_start);
      line_list.points.push_back(line_end);
    }
    static_scan_pub_.publish(occluded_scan);
    marker_front_pub_.publish(line_list);

    visualization_msgs::Marker line_list_occ;
    line_list_occ.header.frame_id = laser_msg_.header.frame_id;
    line_list_occ.header.stamp = laser_msg_.header.stamp;
    line_list_occ.type = visualization_msgs::Marker::LINE_LIST;
    line_list_occ.scale.x = 0.1;
    line_list_occ.color.b = 1.0;
    line_list_occ.color.a = 1.0;

    for (auto it = occluded_clusters.begin(); it != occluded_clusters.end(); ++it){
      geometry_msgs::Point line_start,line_end;
      for (auto map_it = (*it).begin(); map_it != (*it).end(); ++map_it){
        if (map_it == (*it).begin()){
          line_start.x = map_it->second * cos(map_it->first * laser_msg_.angle_increment+M_PI);
          line_start.y = map_it->second * sin(map_it->first * laser_msg_.angle_increment+M_PI);
          line_start.z = 0;
        }
        else if (map_it == --(*it).end()){
          line_end.x = map_it->second * cos(map_it->first * laser_msg_.angle_increment+M_PI);
          line_end.y = map_it->second * sin(map_it->first * laser_msg_.angle_increment+M_PI);
          line_end.z = 0;
        }
      }
      line_list_occ.points.push_back(line_start);
      line_list_occ.points.push_back(line_end);
    }
    marker_occluded_pub_.publish(line_list_occ);
  }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "static_laser_scan_combiner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  StaticLaserScanCombiner combiner(nh, nh_);

  ros::spin();

  return 0;
}
