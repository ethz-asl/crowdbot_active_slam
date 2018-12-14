/**
 *  static_laser_scan_combiner.cpp
 *
 *  Created on: 07.12.2018
 *      Author: Dario Mammolo
 */

#include <static_laser_scan_combiner.h>

using namespace std;

/**
 *  A helper function which creates map id from position information.
 */
int positionToMapId(double x, double y, int width, int height, float resolution){
  int id = (floor(x / resolution) + width / 2) +
           (floor(y / resolution) + height / 2) * width;
  return id;
}

StaticLaserScanCombiner::StaticLaserScanCombiner
  (ros::NodeHandle nh, ros::NodeHandle nh_):nh_(nh), nh_private_(nh_)
{
  ROS_INFO("Started StaticLaserScanCombiner");
  object_detector_ = ObjectDetector(20, 0.01);

  // Subscriber and publisher
  map_sub_ = nh_.subscribe("/occupancy_map", 1, &StaticLaserScanCombiner::mapCallback, this);
  scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>
              (nh_, "/base_scan", 1);
  odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>
              (nh_, "/mobile_base_controller/odom", 1);
  int q = 5; // queue size
  sync_ = new message_filters::Synchronizer<SyncPolicy>
          (SyncPolicy(q), *scan_sub_, *odom_sub_);
  sync_->registerCallback(boost::bind(&StaticLaserScanCombiner::scanOdomCallback,
                                      this, _1, _2));

  static_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("static_combined_scan", 1);
  dynamic_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("dynamic_scan", 1);
  marker_front_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_front", 1);
  marker_occluded_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_occluded", 1);

  // test_pub_ = nh_.advertise<nav_msgs::GridCells>("test", 1);

  get_current_pose_client_ = nh_.serviceClient<crowdbot_active_slam::current_pose>
                             ("/current_pose_node_service");

  begin_time_ = ros::Time::now();
  // Wait until time is not 0
  while (begin_time_.toSec() == 0) begin_time_ = ros::Time::now();
  initialized_first_scan_ = false;

  // Initialize base to laser tf
  tf::StampedTransform base_to_laser_tf_;

  bool got_transform = false;
  while (!got_transform){
    if (true){  // TODO: add using gazebo
      try {
        got_transform = base_to_laser_listener_.waitForTransform("base_link",
                                    "laser", ros::Time(0), ros::Duration(1.0));
        base_to_laser_listener_.lookupTransform("base_link", "laser",
                                              ros::Time(0), base_to_laser_tf_);
      }
      catch (tf::TransformException ex){
        ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
      }
    }
    else {
      try {
        got_transform = base_to_laser_listener_.waitForTransform("base_footprint",
                          "sick_laser_front", ros::Time(0), ros::Duration(1.0));
        base_to_laser_listener_.lookupTransform("base_footprint", "sick_laser_front",
                                               ros::Time(0), base_to_laser_tf_);
      }
      catch (tf::TransformException ex){
        ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
      }
    }
  }

  base_to_laser_ = base_to_laser_tf_;
  laser_to_base_ = base_to_laser_tf_.inverse();
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

    // Publish init scan if enough scan points available, TODO: var for 50
    if (count_unknown < 50){
      static_scan_pub_.publish(init_scan);
      last_node_stamp_ = init_scan.header.stamp;
      initialized_first_scan_ = true;
    }
  }
}

void StaticLaserScanCombiner::scanOdomCallback
     (const sensor_msgs::LaserScan::ConstPtr& scan_msg,
      const nav_msgs::Odometry::ConstPtr& odom_msg){
  // Call latest pose estimate
  if (initialized_first_scan_){
    crowdbot_active_slam::current_pose current_pose_srv;
    if (get_current_pose_client_.call(current_pose_srv)){}
    else ROS_INFO("Current pose service call failed!");

    if (current_pose_srv.response.current_pose.header.stamp ==
        laser_msg_.header.stamp){
      tf::poseStampedMsgToTF(current_pose_srv.response.current_pose,
                             current_pose_tf_);
      tf::poseMsgToTF(odom_msg_.pose.pose, current_odom_tf_);
      map_to_odom_tf_ = current_pose_tf_ * current_odom_tf_.inverse();
      map_to_last_node_tf_ = current_pose_tf_;
    }
  }

  // Save laser and odom msg
  laser_msg_ = *scan_msg;
  odom_msg_ = *odom_msg;

  tf::poseMsgToTF(odom_msg_.pose.pose, current_odom_tf_);
  map_to_latest_tf_ = map_to_odom_tf_ * current_odom_tf_;
  map_to_latest_laser_tf_ = map_to_latest_tf_ * base_to_laser_;

  // Save scan size
  size_t laser_scan_size = laser_msg_.ranges.size();

  if (init_scan_sum_.empty()) init_scan_sum_.resize(laser_scan_size);
  if (!initialized_first_scan_) {
    init_scan_ = laser_msg_;
    initScan(laser_msg_, init_scan_);
  }
  else {
    // Check each scan point if it is a wall from static occupancy map
    dynamic_scan_ = laser_msg_;
    double theta = laser_msg_.angle_min;

    for (size_t i = 0; i < laser_scan_size; i ++){
      geometry_msgs::Point temp_point;
      temp_point.x = laser_msg_.ranges[i] * cos(theta);
      temp_point.y = laser_msg_.ranges[i] * sin(theta);
      temp_point.z = 0;
      tf::Point temp_tf;
      pointMsgToTF(temp_point, temp_tf);
      temp_tf = map_to_latest_laser_tf_ * temp_tf;
      pointTFToMsg(temp_tf, temp_point);
      int id = positionToMapId(temp_point.x, temp_point.y, 2000,
                                  2000, 0.05);
      bool occupied = false;
      if (map_msg_->data[id] > 80) occupied = true;
      if (map_msg_->data[id+1] > 80) occupied = true;
      if (map_msg_->data[id-1] > 80) occupied = true;
      if (map_msg_->data[id+2000] > 80) occupied = true;
      if (map_msg_->data[id-2000] > 80) occupied = true;

      if (occupied == true){
        dynamic_scan_.ranges[i] = 0;
      }
      theta += laser_msg_.angle_increment;
    }

    // Detect and Track objects (ABD + KF)

    dynamic_scan_pub_.publish(dynamic_scan_);
    static_scan_pub_.publish(init_scan_);

    list<map<int, double>> normal_clusters;
    list<map<int, double>> occluded_clusters;
    object_detector_.detectObjectsFromScan(dynamic_scan_,
                                           laser_msg_,
                                           normal_clusters,
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
    // static_scan_pub_.publish(occluded_scan);
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

void StaticLaserScanCombiner::mapCallback
    (const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
  map_msg_ = map_msg;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "static_laser_scan_combiner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  StaticLaserScanCombiner combiner(nh, nh_);

  ros::spin();

  return 0;
}
