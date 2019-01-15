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

std::vector<unsigned int> getNeighbourXCells(unsigned int id,
                int map_width, int map_height, unsigned int n_cells){
  std::vector<unsigned int> neighbour_vec;

  int br_corner = id - n_cells - n_cells * map_width;
  for (unsigned int i = 0; i <= 2 * n_cells; i++){
    for (unsigned int j = 0; j <= 2 * n_cells; j++){
      int new_id = br_corner + j + i * map_width;
      if (new_id >= 0 && new_id <= map_width * map_height && new_id != int(id)){
        neighbour_vec.push_back(new_id);
      }
    }
  }
  return neighbour_vec;
}

vector<geometry_msgs::Point> getClusterInLaserFrame(map<int, double>& cluster,
                                            sensor_msgs::LaserScan& laser_msg){
  vector<geometry_msgs::Point> cluster_vector;
  for (auto cluster_it = cluster.begin(); cluster_it != cluster.end(); ++cluster_it){
    geometry_msgs::Point temp_point;
    temp_point.x = cluster_it->second * cos(cluster_it->first *
                   laser_msg.angle_increment + laser_msg.angle_min);
    temp_point.y = cluster_it->second * sin(cluster_it->first *
                   laser_msg.angle_increment + laser_msg.angle_min);
    temp_point.z = 0;
    cluster_vector.push_back(temp_point);
  }
  return cluster_vector;
}

geometry_msgs::Point getMean(vector<geometry_msgs::Point> cluster){
  double x = 0, y = 0;
  for (auto cluster_it = cluster.begin(); cluster_it != cluster.end(); ++cluster_it){
    x += cluster_it->x;
    y += cluster_it->y;
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
  map_sub_ = nh_.subscribe("/occupancy_map_for_static_scan_comb", 1,
                           &StaticLaserScanCombiner::mapCallback, this);
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

  get_current_pose_client_ = nh_.serviceClient<crowdbot_active_slam::current_pose>
                             ("/current_pose_node_service");

  begin_time_ = ros::Time::now();
  // Wait until time is not 0
  while (begin_time_.toSec() == 0) begin_time_ = ros::Time::now();
  initialized_first_scan_ = false;
  map_callback_initialized_ = false;

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
  std_dev_process_ = 0.1; // for what?
  std_dev_range_ = 0.01;
  std_dev_theta_ = 0.01;
  kalman_filter_ = KalmanFilter(std_dev_range_, std_dev_theta_);
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
    if (count_unknown < 100){
      static_scan_pub_.publish(init_scan);
      initialized_first_scan_ = true;
      ROS_INFO("Scan initialised!");
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
    if (map_callback_initialized_ == false) return;
    // Check each scan point if it is a wall from static occupancy map
    dynamic_scan_ = laser_msg_;
    static_scan_ = laser_msg_;
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
      // if (map_msg_.data[id] > 80) occupied = true;
      // if (map_msg_.data[id+1] > 80) occupied = true;
      // if (map_msg_.data[id-1] > 80) occupied = true;
      // if (map_msg_.data[id+2000] > 80) occupied = true;
      // if (map_msg_.data[id-2000] > 80) occupied = true;
      // if (map_msg_.data[id+1999] > 80) occupied = true;
      // if (map_msg_.data[id+2001] > 80) occupied = true;
      // if (map_msg_.data[id-1999] > 80) occupied = true;
      // if (map_msg_.data[id-2001] > 80) occupied = true;
      std::vector<unsigned int> neighbours;
      neighbours = getNeighbourXCells(id, 2000, 2000, 2);
      for (int iter = 0; iter < neighbours.size(); iter++){
        if (map_msg_.data[neighbours[iter]] > 80) occupied = true;
      }

      if (occupied == true){
        dynamic_scan_.ranges[i] = 0;
      }
      theta += laser_msg_.angle_increment;
    }

    // Detect and Track objects (ABD + KF)
    // Detect objects
    // TODO: define a cluster class
    list<map<int, double>> free_clusters;
    list<map<int, double>> occluded_clusters;
    object_detector_.detectObjectsFromScan(dynamic_scan_,
                                           laser_msg_,
                                           free_clusters,
                                           occluded_clusters);

    // Track objects
    free_means_.clear();
    occluded_means_.clear();
    vector<vector<geometry_msgs::Point>> free_cluster_vectors;
    vector<vector<geometry_msgs::Point>> occluded_cluster_vectors;
    for (auto cluster_it = free_clusters.begin(); cluster_it != free_clusters.end(); ++cluster_it){
      geometry_msgs::Point temp_mean;
      tf::Point temp_mean_tf;
      free_cluster_vectors.push_back(getClusterInLaserFrame(*(cluster_it), laser_msg_));
      temp_mean = getMean(free_cluster_vectors.back());
      // Transform mean to Map frame
      pointMsgToTF(temp_mean, temp_mean_tf);
      temp_mean_tf = map_to_latest_laser_tf_ * temp_mean_tf;
      pointTFToMsg(temp_mean_tf, temp_mean);
      free_means_.push_back(temp_mean);
    }
    for (auto cluster_it = occluded_clusters.begin(); cluster_it != occluded_clusters.end(); ++cluster_it){
      geometry_msgs::Point temp_mean;
      tf::Point temp_mean_tf;
      occluded_cluster_vectors.push_back(getClusterInLaserFrame(*(cluster_it), laser_msg_));
      temp_mean = getMean(occluded_cluster_vectors.back());
      // Transform mean to Map frame
      pointMsgToTF(temp_mean, temp_mean_tf);
      temp_mean_tf = map_to_latest_laser_tf_ * temp_mean_tf;
      pointTFToMsg(temp_mean_tf, temp_mean);
      occluded_means_.push_back(temp_mean);
    }

    // Kalman Filter update with new delta_t
    kalman_filter_.updateProcessMatrices(prev_delta_t_);

    // Check if cluster already tracked, if not new track
    if (!tracked_objects_.empty()){
      for (int i = 0; i < tracked_objects_.size(); i++){
        // Prediction
        tracked_objects_[i].state_mean =
          kalman_filter_.prediction(tracked_objects_[i].state_mean);
        tracked_objects_[i].state_var =
          kalman_filter_.prediction(tracked_objects_[i].state_var);

        int id = -1;
        double distance = 10;
        std::vector<geometry_msgs::Point> all_means = free_means_;
        all_means.insert(all_means.end(), occluded_means_.begin(), occluded_means_.end());
        for (size_t j = 0; j < all_means.size(); j++){
          // Check distance (later try mahalanobis)
          double temp_distance =
          sqrt(pow(tracked_objects_[i].state_mean[0] - all_means[j].x, 2) +
               pow(tracked_objects_[i].state_mean[1] - all_means[j].y, 2));
          // Check if current distance smaller
          if (temp_distance < 0.3 && temp_distance < distance){
            id = j;
            distance = temp_distance;
          }
        }
        // Check if found assignement
        if (id != -1){
          tracked_objects_[i].counter_not_seen = 0;
          // Update tracked object
          kalman_filter_.updateKalmanGain(tracked_objects_[i].state_var);

          // Get measurement
          Eigen::Vector2d temp_z_m;
          temp_z_m << all_means[id].x, all_means[id].y;

          tracked_objects_[i].state_mean =
              kalman_filter_.update(tracked_objects_[i].state_mean, temp_z_m);

          tracked_objects_[i].state_var =
              kalman_filter_.update(tracked_objects_[i].state_var, temp_z_m);

          if (tracked_objects_[i].dynamic_or_static == "static"){
            int map_id = positionToMapId(temp_z_m[0], temp_z_m[1], 2000,
                                        2000, 0.05);
            if (map_msg_.data[map_id] == -1){
              // If object is in unknown space, assume it is static
              tracked_objects_[i].dynamic_or_static = "static";
            }
            else {
              // If object is in free space, assume it is dynamic
              tracked_objects_[i].dynamic_or_static = "dynamic";
            }
          }

          // Delete measurement
          if (id >= free_means_.size()){
            tracked_objects_[i].current_occlusion = "occluded";

            auto it_delete = occluded_clusters.begin();
            std::advance(it_delete, id - free_means_.size());
            if (tracked_objects_[i].dynamic_or_static == "static"){
              for (auto cluster_it = (*it_delete).begin();
                   cluster_it != (*it_delete).end(); ++cluster_it){
                dynamic_scan_.ranges[cluster_it->first] = 0;
              }
            }
            else {
              for (auto cluster_it = (*it_delete).begin();
                   cluster_it != (*it_delete).end(); ++cluster_it){
                static_scan_.ranges[cluster_it->first] = 0;
              }
            }
            occluded_clusters.erase(it_delete);

            tracked_objects_[i].saveCluster(occluded_cluster_vectors[id - free_means_.size()]);
            occluded_cluster_vectors.erase(occluded_cluster_vectors.begin() + id - free_means_.size());
            occluded_means_.erase(occluded_means_.begin() + id - free_means_.size());
          }
          else {
            tracked_objects_[i].current_occlusion = "free";

            auto it_delete = free_clusters.begin();
            std::advance(it_delete, id);
            if (tracked_objects_[i].dynamic_or_static == "static"){
              for (auto cluster_it = (*it_delete).begin();
                   cluster_it != (*it_delete).end(); ++cluster_it){
                dynamic_scan_.ranges[cluster_it->first] = 0;
              }
            }
            else {
              for (auto cluster_it = (*it_delete).begin();
                   cluster_it != (*it_delete).end(); ++cluster_it){
                static_scan_.ranges[cluster_it->first] = 0;
              }
            }
            free_clusters.erase(it_delete);

            tracked_objects_[i].saveCluster(free_cluster_vectors[id]);
            free_cluster_vectors.erase(free_cluster_vectors.begin() + id);
            free_means_.erase(free_means_.begin() + id);
          }
        }
        else {
          if (tracked_objects_[i].counter_not_seen < 10){
            tracked_objects_[i].counter_not_seen += 1;
          }
          else {
            tracked_objects_.erase(tracked_objects_.begin() + i);
            i -= 1;
          }
        }
      }
    }

    std::vector<geometry_msgs::Point> all_means = free_means_;
    all_means.insert(all_means.end(), occluded_means_.begin(), occluded_means_.end());
    if (!all_means.empty()){
      // Generate new tracked objects
      for (size_t i = 0; i < all_means.size(); i++){
        TrackedObject temp_tracked_object(all_means[i].x, all_means[i].y);

        // Check if object is dynamic or static
        int id = positionToMapId(all_means[i].x, all_means[i].y, 2000,
                                    2000, 0.05);

        auto it_delete = occluded_clusters.begin();
        if (i >= free_means_.size()){
          it_delete = occluded_clusters.begin();
          std::advance(it_delete, i - free_means_.size());
        }
        else{
          it_delete = free_clusters.begin();
          std::advance(it_delete, i);
        }

        if (map_msg_.data[id] == -1){
          // If object is in unknown space, assume it is static
          temp_tracked_object.dynamic_or_static = "static";

          for (auto cluster_it = (*it_delete).begin();
               cluster_it != (*it_delete).end(); ++cluster_it){
            dynamic_scan_.ranges[cluster_it->first] = 0;
          }
        }
        else {
          // If object is in free space, assume it is dynamic
          temp_tracked_object.dynamic_or_static = "dynamic";

          for (auto cluster_it = (*it_delete).begin();
               cluster_it != (*it_delete).end(); ++cluster_it){
            static_scan_.ranges[cluster_it->first] = 0;
          }
        }

        // Save occlusion and cluster
        if (i >= free_means_.size()){
          temp_tracked_object.current_occlusion = "occluded";
          temp_tracked_object.saveCluster(occluded_cluster_vectors[i - free_means_.size()]);
        }
        else {
          temp_tracked_object.current_occlusion = "free";
          temp_tracked_object.saveCluster(free_cluster_vectors[i]);
        }

        tracked_objects_.push_back(temp_tracked_object);
      }
    }

    // Publish means of tracked objects
    nav_msgs::GridCells tracked_objects_msg;
    tracked_objects_msg.header.frame_id = "map";
    tracked_objects_msg.cell_width = 0.2;
    tracked_objects_msg.cell_height = 0.2;
    for (size_t i = 0; i < tracked_objects_.size(); i++){
      geometry_msgs::Point temp_point;
      temp_point.x = tracked_objects_[i].state_mean[0];
      temp_point.y = tracked_objects_[i].state_mean[1];
      temp_point.z = 0;
      tracked_objects_msg.cells.push_back(temp_point);
    }
    tracked_objects_pub_.publish(tracked_objects_msg);

    // Update delta_t
    curr_node_stamp_ = laser_msg_.header.stamp;
    prev_delta_t_ = (curr_node_stamp_ - prev_node_stamp_).toSec();
    prev_node_stamp_ = curr_node_stamp_;

    dynamic_scan_pub_.publish(dynamic_scan_);
    static_scan_pub_.publish(static_scan_);
  }
}

void StaticLaserScanCombiner::mapCallback
    (const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
  map_msg_ = *map_msg;
  map_callback_initialized_ = true;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "static_laser_scan_combiner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  StaticLaserScanCombiner combiner(nh, nh_);

  ros::spin();

  return 0;
}
