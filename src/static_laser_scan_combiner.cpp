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
  static_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("not_occluded_scan", 1);
  scan_sub_ = nh_.subscribe("/base_scan", 1, &StaticLaserScanCombiner::scanCallback, this);
  marker_front_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_front", 1);
  marker_occluded_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_occluded", 1);
}

StaticLaserScanCombiner::~StaticLaserScanCombiner(){}

void StaticLaserScanCombiner::scanCallback
     (const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  laser_msg_ = *scan_msg;

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


int main(int argc, char **argv){
  ros::init(argc, argv, "static_laser_scan_combiner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  StaticLaserScanCombiner combiner(nh, nh_);

  ros::spin();

  return 0;
}
