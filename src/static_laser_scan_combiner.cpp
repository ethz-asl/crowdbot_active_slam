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

geometry_msgs::Point getMean(map<int, double>& cluster,
                             sensor_msgs::LaserScan& laser_msg){
  double x = 0, y = 0;
  for (auto cluster_it = cluster.begin(); cluster_it != cluster.end(); ++cluster_it){
    x += cluster_it->second * cos(cluster_it->first * laser_msg.angle_increment
         + laser_msg.angle_min);
    y += cluster_it->second * sin(cluster_it->first * laser_msg.angle_increment
         + laser_msg.angle_min);
  }
  geometry_msgs::Point mean;
  mean.x = x / cluster.size();
  mean.y = y / cluster.size();
  mean.z = 0;
  return mean;
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
  tracked_objects_pub_ = nh_.advertise<nav_msgs::GridCells>("tracked_objects", 1);

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

  // KF init variables
  std_dev_process_ = 0.1;
  std_dev_range_ = 0.01;
  std_dev_theta_ = 0.01;
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
    curr_node_stamp_ = laser_msg_.header.stamp;
    prev_delta_t_ = (curr_node_stamp_ - prev_node_stamp_).toSec();
    initScan(laser_msg_, init_scan_);
    prev_node_stamp_ = curr_node_stamp_;
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

    // Detect objects
    list<map<int, double>> normal_clusters;
    list<map<int, double>> occluded_clusters;
    object_detector_.detectObjectsFromScan(dynamic_scan_,
                                           laser_msg_,
                                           normal_clusters,
                                           occluded_clusters);

    // Track objects
    normal_means_.clear();
    occluded_means_.clear();
    for (auto cluster_it = normal_clusters.begin(); cluster_it != normal_clusters.end(); ++cluster_it){
      geometry_msgs::Point temp_mean;
      tf::Point temp_mean_tf;
      temp_mean = getMean(*(cluster_it), laser_msg_);
      pointMsgToTF(temp_mean, temp_mean_tf);
      temp_mean_tf = map_to_latest_laser_tf_ * temp_mean_tf;
      pointTFToMsg(temp_mean_tf, temp_mean);
      normal_means_.push_back(temp_mean);
    }
    for (auto cluster_it = occluded_clusters.begin(); cluster_it != occluded_clusters.end(); ++cluster_it){
      geometry_msgs::Point temp_mean;
      tf::Point temp_mean_tf;
      temp_mean = getMean(*(cluster_it), laser_msg_);
      pointMsgToTF(temp_mean, temp_mean_tf);
      temp_mean_tf = map_to_latest_laser_tf_ * temp_mean_tf;
      pointTFToMsg(temp_mean_tf, temp_mean);
      occluded_means_.push_back(temp_mean);
    }

    // Define Process matrices
    Eigen::MatrixXd A_prev(4, 4);
    A_prev << 1, 0, prev_delta_t_, 0,
              0, 1, 0, prev_delta_t_,
              0, 0, 1, 0,
              0, 0, 0, 1;

    Eigen::MatrixXd Q_prev(4, 4);
    Q_prev << pow(prev_delta_t_, 4) / 4, 0, pow(prev_delta_t_, 3) / 2, 0,
              0, pow(prev_delta_t_, 4) / 4, 0, pow(prev_delta_t_, 3) / 2,
              pow(prev_delta_t_, 3) / 2, 0, pow(prev_delta_t_, 2), 0,
              0, pow(prev_delta_t_, 3) / 2, 0, pow(prev_delta_t_, 2);

    // Measurement matrices
    // Currently using easy approach and std dev are for x and y
    Eigen::MatrixXd H_prev(2, 4);
    H_prev << 1, 0, 0, 0,
              0, 1, 0, 0;

    Eigen::MatrixXd R_prev(2, 2);
    R_prev << pow(std_dev_range_, 2), 0,
              0, pow(std_dev_theta_, 2);

    // Check if cluster already tracked, if not new track
    std::vector<geometry_msgs::Point> all_means = normal_means_;
    all_means.insert(all_means.end(), occluded_means_.begin(), occluded_means_.end());
    if (!tracked_objects_mean_.empty()){
      for (size_t i = 0; i < tracked_objects_mean_.size(); i++){
        int id = -1;
        double distance = 10;
        for (size_t j = 0; j < all_means.size(); j++){
          // Check distance (later try mahalanobis)
          double temp_distance =
          sqrt(pow(tracked_objects_mean_[i][0] - all_means[j].x, 2) +
               pow(tracked_objects_mean_[i][1] - all_means[j].y, 2));
          // Check if current distance smaller
          if (temp_distance < 0.3 && temp_distance < distance){
            id = j;
            distance = temp_distance;
          }
        }
        // Check if found assignement
        if (id != -1){
          tracked_objects_counter_[i] = 0;
          // Update tracked object (Prediction+Update)
          // Prediction
          tracked_objects_mean_[i] = A_prev * tracked_objects_mean_[i];
          tracked_objects_var_[i] = A_prev * tracked_objects_var_[i] * A_prev.transpose()
                                    + Q_prev;

          // Update
          Eigen::MatrixXd temp_gain(4, 2);
          temp_gain = tracked_objects_var_[i] * H_prev.transpose() *
            (H_prev * tracked_objects_var_[i] * H_prev.transpose() +
             R_prev).inverse();

          Eigen::Vector2d temp_z_m;
          temp_z_m << all_means[id].x, all_means[id].y;

          tracked_objects_mean_[i] = tracked_objects_mean_[i] + temp_gain *
            (temp_z_m - H_prev * tracked_objects_mean_[i]);

          tracked_objects_var_[i] = (Eigen::MatrixXd::Identity(4, 4) -
            temp_gain * H_prev) * tracked_objects_var_[i];

          // Delete measurement
          all_means.erase(all_means.begin() + id);
        }
        else {
          if (tracked_objects_counter_[i] < 5){
            // Update tracked object (Prediction)
            tracked_objects_mean_[i] = A_prev * tracked_objects_mean_[i];
            tracked_objects_var_[i] = A_prev * tracked_objects_var_[i] * A_prev.inverse()
                                   + Q_prev;
            tracked_objects_counter_[i] += 1;
          }
          else {
            tracked_objects_mean_.erase(tracked_objects_mean_.begin() + i);
            tracked_objects_var_.erase(tracked_objects_var_.begin() + i);
            tracked_objects_counter_.erase(tracked_objects_counter_.begin() + i);
          }
        }
      }
    }

    if (!all_means.empty()){
      // Generate new tracked objects
      for (size_t i = 0; i < all_means.size(); i++){
        Eigen::Vector4d temp_track_mean;
        temp_track_mean << all_means[i].x, all_means[i].y, 0, 0;

        Eigen::MatrixX4d temp_track_var(4, 4);
        temp_track_var << 1, 0, 0, 0,
                          0, 1, 0, 0,
                          0, 0, 1, 0,
                          0, 0, 0, 1;

        tracked_objects_mean_.push_back(temp_track_mean);
        tracked_objects_var_.push_back(temp_track_var);
        tracked_objects_counter_.push_back(0);
      }
    }

    // Publish means of tracked objects
    nav_msgs::GridCells tracked_objects_msg;
    tracked_objects_msg.header.frame_id = "map";
    tracked_objects_msg.cell_width = 0.2;
    tracked_objects_msg.cell_height = 0.2;
    for (size_t i = 0; i < tracked_objects_mean_.size(); i++){
      geometry_msgs::Point temp_point;
      temp_point.x = tracked_objects_mean_[i][0];
      temp_point.y = tracked_objects_mean_[i][1];
      temp_point.z = 0;
      tracked_objects_msg.cells.push_back(temp_point);
    }
    tracked_objects_pub_.publish(tracked_objects_msg);

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
          line_start.x = map_it->second * cos(map_it->first * laser_msg_.angle_increment
                                              + laser_msg_.angle_min);
          line_start.y = map_it->second * sin(map_it->first * laser_msg_.angle_increment
                                              + laser_msg_.angle_min);
          line_start.z = 0;
        }
        else if (map_it == --(*it).end()){
          line_end.x = map_it->second * cos(map_it->first * laser_msg_.angle_increment
                                            + laser_msg_.angle_min);
          line_end.y = map_it->second * sin(map_it->first * laser_msg_.angle_increment
                                            + laser_msg_.angle_min);
          line_end.z = 0;
        }
      }
      line_list_occ.points.push_back(line_start);
      line_list_occ.points.push_back(line_end);
    }
    marker_occluded_pub_.publish(line_list_occ);

    // Update delta_t
    curr_node_stamp_ = laser_msg_.header.stamp;
    prev_delta_t_ = (curr_node_stamp_ - prev_node_stamp_).toSec();
    prev_node_stamp_ = curr_node_stamp_;
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
