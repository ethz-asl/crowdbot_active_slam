/**
 *  static_laser_scan_combiner.cpp
 *
 *  Created on: 07.12.2018
 *      Author: Dario Mammolo
 */

#include <static_scan_extractor.h>

using namespace std;

/**
 *  A helper function which creates tf msg from x, y, theta information.
 */
tf::Transform xythetaToTF(double x, double y, double theta){
  tf::Transform xytheta_tf;
  xytheta_tf.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  xytheta_tf.setRotation(q);

  return xytheta_tf;
}

/**
 *  A helper function which creates map id from position information.
 */
int positionToMapId(double x, double y, int width, int height, float resolution){
  int id = (floor(x / resolution) + width / 2) +
           (floor(y / resolution) + height / 2) * width;
  return id;
}

/**
 *  A helper function which gets map IDs around it's cell in a square region
 *  with distance x.
 */
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

/**
 *  A helper function which transforms a cluster of laser scans to a cluster
 *  in the laser frame.
 */
vector<geometry_msgs::Point> StaticScanExtractor::getClusterInLaserFrame
                             (map<int, double>& cluster){
  vector<geometry_msgs::Point> cluster_vector;
  for (auto cluster_it = cluster.begin(); cluster_it != cluster.end(); ++cluster_it){
    geometry_msgs::Point temp_point;
    temp_point.x = cluster_it->second * cos(cluster_it->first *
                   scan_angle_increment_ + scan_angle_min_);
    temp_point.y = cluster_it->second * sin(cluster_it->first *
                   scan_angle_increment_ + scan_angle_min_);
    temp_point.z = 0;
    cluster_vector.push_back(temp_point);
  }
  return cluster_vector;
}

/**
 *  A helper function which gets the mean from a cluster.
 */
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

void StaticScanExtractor::laserScanToLDP(sensor_msgs::LaserScan& scan_msg, LDP& ldp){
  unsigned int n = scan_ranges_size_;
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++){
    double r = scan_msg.ranges[i];

    if (r > scan_range_min_ && r < 100){
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    }
    else if (r == 0){
      ldp->valid[i] = 0;
      ldp->readings[i] = 0;
    }
    else if (r <= scan_range_min_){
      ldp->valid[i] = 0;
      ldp->readings[i] = 0;
    }
    else{
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;
    }
    ldp->theta[i] = scan_msg.angle_min + i * scan_angle_increment_;
    ldp->cluster[i] = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n - 1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

StaticScanExtractor::StaticScanExtractor
  (ros::NodeHandle nh, ros::NodeHandle nh_):nh_(nh), nh_private_(nh_)
{
  ROS_INFO("Started StaticScanExtractor");

  // Object detector
  nh_private_.param<double>("ABD_lambda", abd_lambda_, 10);
  nh_private_.param<double>("ABD_sigma", abd_sigma_, 0.01);
  object_detector_ = ObjectDetector(abd_lambda_, abd_sigma_);

  // Subscriber
  map_sub_ = nh_.subscribe("/map_for_static_scan_extractor", 1,
                           &StaticScanExtractor::mapCallback, this);
  current_pose_sub_ = nh_.subscribe("/current_node_estimate", 1,
                           &StaticScanExtractor::poseCallback, this);

  nh_private_.param<std::string>("scan_callback_topic", scan_callback_topic_,
                                 "/base_scan");
  scan_sub_ = nh_.subscribe(scan_callback_topic_, 5,
                            &StaticScanExtractor::scanCallback, this);

  // Publisher
  static_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("static_scan", 1);
  dynamic_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("dynamic_scan", 1);
  moving_objects_pub_ = nh_.advertise<nav_msgs::GridCells>("moving_objects", 1);
  static_objects_pub_ = nh_.advertise<nav_msgs::GridCells>("static_objects", 1);
  unknown_objects_pub_ = nh_.advertise<nav_msgs::GridCells>("unknown_objects", 1);
  debug_scan_pub_ = nh_.advertise<nav_msgs::GridCells>("debug_scans", 1);
  dyn_obstacles_pub_ = nh_.advertise<costmap_converter::ObstacleArrayMsg>("dyn_obstacles", 1);

  // Service
  get_current_pose_client_ = nh_.serviceClient<crowdbot_active_slam::current_pose>
                             ("/current_pose_node_service");

  // Get params from other nodes
  nh_.getParam("/graph_optimisation/robot_name", robot_name_);
  nh_.getParam("/graph_optimisation/map_width", map_width_);
  nh_.getParam("/graph_optimisation/map_height", map_height_);
  nh_.getParam("/graph_optimisation/map_resolution", map_resolution_);

  begin_time_ = ros::Time::now();
  // Wait until time is not 0
  while (begin_time_.toSec() == 0) begin_time_ = ros::Time::now();
  initialised_first_scan_ = false;
  map_callback_initialised_ = false;

  // Initialize base to laser tf
  tf::StampedTransform base_to_laser_tf;

  bool got_transform = false;
  while (!got_transform){
    if (robot_name_ == "pioneer_sim"){
      try {
        got_transform = base_to_laser_listener_.waitForTransform("base_link",
                                    "laser", ros::Time(0), ros::Duration(1.0));
        base_to_laser_listener_.lookupTransform("base_link", "laser",
                                              ros::Time(0), base_to_laser_tf);
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
                                               ros::Time(0), base_to_laser_tf);
      }
      catch (tf::TransformException ex){
        ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
      }
    }
  }

  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_tf.inverse();

  // KF init variables
  nh_private_.param<double>("kf_std_dev_process", std_dev_process_, 1);
  nh_private_.param<double>("kf_std_dev_range", std_dev_range_, 0.01);
  nh_private_.param<double>("kf_std_dev_theta", std_dev_theta_, 0.01);

  kalman_filter_ = KalmanFilter(std_dev_range_, std_dev_theta_, std_dev_process_);

  /*
  Init some sm_icp parameters (Most values are taken from the
  ROS laser scan matcher package)
  */
  /*
  Relative position from laser to robot (Here we set this to zero as
  we consider this separately)
  */
  sm_icp_params_.laser[0] = 0.0;
  sm_icp_params_.laser[1] = 0.0;
  sm_icp_params_.laser[2] = 0.0;

  sm_icp_params_.max_angular_correction_deg = 25.0;
  sm_icp_params_.max_linear_correction = 0.3;
  sm_icp_params_.max_iterations = 10;
  sm_icp_params_.epsilon_xy = 0.000001;
  sm_icp_params_.epsilon_theta = 0.000001;
  sm_icp_params_.max_correspondence_dist = 0.5;
  sm_icp_params_.use_corr_tricks = 1;
  sm_icp_params_.restart = 0;
  sm_icp_params_.restart_threshold_mean_error = 0.0357;
  sm_icp_params_.restart_dt = 0.03;
  sm_icp_params_.restart_dtheta = 0.1;
  sm_icp_params_.outliers_maxPerc = 0.97;
  sm_icp_params_.outliers_adaptive_order = 0.97;
  sm_icp_params_.outliers_adaptive_mult = 2.0;
  sm_icp_params_.outliers_remove_doubles = 1;
  sm_icp_params_.clustering_threshold = 0.25;
  sm_icp_params_.orientation_neighbourhood = 20;
  sm_icp_params_.do_alpha_test = 0;
  sm_icp_params_.do_alpha_test_thresholdDeg = 20.0;
  sm_icp_params_.do_visibility_test = 0;
  sm_icp_params_.use_point_to_line_distance = 1;
  sm_icp_params_.use_ml_weights = 0;
  sm_icp_params_.use_sigma_weights = 0;
  sm_icp_params_.debug_verify_tricks = 0;
  sm_icp_params_.sigma = 0.01;
  sm_icp_params_.do_compute_covariance = 0;

  // Other params
  nh_private_.param<int>("init_scan_unknown_pt", init_scan_unknown_pt_, 100);
  nh_private_.param<int>("init_min_time", init_min_time_, 4);
  nh_private_.param<int>("wall_threshold", wall_threshold_, 80);
  nh_private_.param<double>("association_radius", association_radius_, 0.5);
  association_radius_sq_ = association_radius_ * association_radius_;
  nh_private_.param<int>("unknown_since_threshold", unknown_since_threshold_, 30);
  nh_private_.param<double>("static_size_threshold", static_size_threshold_, 0.7);
  static_size_threshold_sq_ = static_size_threshold_ * static_size_threshold_;
  nh_private_.param<double>("dynamic_vel_threshold", dynamic_vel_threshold_, 0.13);
  dynamic_vel_threshold_sq_ = dynamic_vel_threshold_ * dynamic_vel_threshold_;
  nh_private_.param<int>("not_seen_threshold", not_seen_threshold_, 10);
  nh_private_.param<int>("upsampling_factor", upsampling_factor_, 1);
  nh_private_.param<int>("static_cell_range", static_cell_range_, 1);
  nh_private_.param<int>("min_cluster_size_for_moving", min_cluster_size_for_moving_, 5);
  nh_private_.param<int>("min_velocity_count", min_velocity_count_, 8);

  // Init param, which is needed to check if ldp already initialised or not
  first_ldp_ = true;
  current_pose_intialised_ = false;

  laser_diff_tf_ = xythetaToTF(0, 0, 0);
}

StaticScanExtractor::~StaticScanExtractor(){}

void StaticScanExtractor::initScan(sensor_msgs::LaserScan& laser_msg,
                                   sensor_msgs::LaserScan& init_scan){
  // Sum and sort scans
  for (size_t i = 0; i < scan_ranges_size_; i++){
    init_scan_sum_[i].push_back(laser_msg.ranges[i]);
    sort(init_scan_sum_[i].begin(), init_scan_sum_[i].end());
  }

  // If passed time is higher than init_min_time_
  ros::Time end_time = ros::Time::now();
  double diff_time = (end_time - begin_time_).toSec();
  if (diff_time > init_min_time_){
    int count_unknown = 0;
    double temp_median;
    size_t scan_sum_size = init_scan_sum_[0].size();
    for (size_t j = 0; j < scan_ranges_size_; j++){
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

    // Publish init scan if enough scan points available
    if (count_unknown < init_scan_unknown_pt_){
      static_scan_pub_.publish(init_scan);
      initialised_first_scan_ = true;
      ROS_INFO("Scan initialised!");
    }
  }
}

void StaticScanExtractor::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
  current_pose_ = *pose_msg;
  current_pose_intialised_ = true;
}

void StaticScanExtractor::scanCallback
     (const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  // Call latest pose estimate
  if (initialised_first_scan_ && current_pose_intialised_){
    if (first_ldp_) current_pose_stamp_ = current_pose_.header.stamp;
    // Check that they have the same stamp
    if (current_pose_.header.stamp > current_pose_stamp_ || first_ldp_){
      tf::poseStampedMsgToTF(current_pose_, current_pose_tf_);
      current_pose_stamp_ = current_pose_.header.stamp;

      if (!first_ldp_){
        // Free memory for avoiding memory leaking
        ld_free(prev_node_ldp_);
      }
      else first_ldp_ = false;

      // Find static scan corresponding to time stamp
      while(current_pose_stamp_ != static_scan_queue_.front().header.stamp){
        static_scan_queue_.pop();
      }

      // Save previous static_scan as LDP
      laserScanToLDP(static_scan_queue_.front(), prev_node_ldp_);
      prev_node_ldp_->estimate[0] = 0.0;
      prev_node_ldp_->estimate[1] = 0.0;
      prev_node_ldp_->estimate[2] = 0.0;

      // Reset first laser tf guess
      laser_diff_tf_ = xythetaToTF(0, 0, 0);
    }
  }

  // Save laser msg
  static_scan_ = *scan_msg;
  dynamic_scan_ = *scan_msg;

  if (!initialised_first_scan_){
    // Init scan params
    scan_ranges_size_ = static_scan_.ranges.size();
    scan_angle_increment_ = static_scan_.angle_increment;
    scan_range_min_ = static_scan_.range_min;
    scan_range_max_ = static_scan_.range_max;
    scan_angle_min_ = static_scan_.angle_min;

    // Init sm icp params
    sm_icp_params_.min_reading = scan_range_min_;
    sm_icp_params_.max_reading = scan_range_max_;

    // Check if init_scan_sum_ not initialised yet
    if (init_scan_sum_.empty()) init_scan_sum_.resize(scan_ranges_size_);

    // Init previous node stamp variable for later kf matrices update
    prev_node_stamp_ = static_scan_.header.stamp;

    // Init init_scan
    init_scan_ = static_scan_;
    initScan(static_scan_, init_scan_);
    static_scan_ = init_scan_;
    static_scan_queue_.push(static_scan_);
  }
  else {
    if (!current_pose_intialised_) return;
    // If map callback not initialised return
    if (map_callback_initialised_ == false) return;

    // Transform current scan to LDP
    laserScanToLDP(static_scan_, current_ldp_);
    current_ldp_->estimate[0] = 0.0;
    current_ldp_->estimate[1] = 0.0;
    current_ldp_->estimate[2] = 0.0;

    // Do scan matching of current and previous node scan
    sm_icp_params_.laser_ref = prev_node_ldp_;
    sm_icp_params_.laser_sens = current_ldp_;
    sm_icp_params_.first_guess[0] = laser_diff_tf_.getOrigin().getX();
    sm_icp_params_.first_guess[1] = laser_diff_tf_.getOrigin().getY();
    sm_icp_params_.first_guess[2] = tf::getYaw(laser_diff_tf_.getRotation());

    sm_icp(&sm_icp_params_, &sm_icp_result_);

    // Get result as tf
    laser_diff_tf_ = xythetaToTF(sm_icp_result_.x[0],
                                 sm_icp_result_.x[1],
                                 sm_icp_result_.x[2]);

    // Transform to base_link frame and get map to latest laser tf
    tf::Transform pose_diff_tf = base_to_laser_ * laser_diff_tf_ * laser_to_base_;
    map_to_latest_tf_ = current_pose_tf_ * pose_diff_tf;
    map_to_latest_laser_tf_ = map_to_latest_tf_ * base_to_laser_;

    // Check each scan point if it is a wall from static occupancy map
    double theta = scan_angle_min_;
    for (size_t i = 0; i < scan_ranges_size_; i ++){
      geometry_msgs::Point temp_point;
      temp_point.x = static_scan_.ranges[i] * cos(theta);
      temp_point.y = static_scan_.ranges[i] * sin(theta);
      temp_point.z = 0;
      tf::Point temp_tf;
      pointMsgToTF(temp_point, temp_tf);
      temp_tf = map_to_latest_laser_tf_ * temp_tf;
      pointTFToMsg(temp_tf, temp_point);

      int id = positionToMapId(temp_point.x, temp_point.y, map_width_,
                               map_height_, map_resolution_);

      // Get neighbours around current scan point and check if there is a wall
      bool occupied = false;
      std::vector<unsigned int> neighbours;
      neighbours = getNeighbourXCells(id, map_width_, map_height_, static_cell_range_);
      for (int iter = 0; iter < neighbours.size(); iter++){
        if (map_msg_.data[neighbours[iter]] > wall_threshold_) occupied = true;
      }
      // If it is a wall cell set as zero
      if (occupied == true){
        dynamic_scan_.ranges[i] = 0;
      }
      theta += scan_angle_increment_;
    }

    // Detect and Track objects (ABD + KF)
    // TODO: define a cluster class
    list<map<int, double>> free_clusters;
    list<map<int, double>> occluded_clusters;
    object_detector_.detectObjectsFromScan(dynamic_scan_,
                                           static_scan_,
                                           free_clusters,
                                           occluded_clusters,
                                           upsampling_factor_);

    // get means and save clusters for later
    free_means_.clear();
    occluded_means_.clear();
    free_cluster_vectors_.clear();
    occluded_cluster_vectors_.clear();
    for (auto cluster_it = free_clusters.begin(); cluster_it != free_clusters.end(); ++cluster_it){
      geometry_msgs::Point temp_mean;
      tf::Point temp_mean_tf;
      free_cluster_vectors_.push_back(getClusterInLaserFrame(*(cluster_it)));
      temp_mean = getMean(free_cluster_vectors_.back());
      // Transform mean to Map frame
      pointMsgToTF(temp_mean, temp_mean_tf);
      temp_mean_tf = map_to_latest_laser_tf_ * temp_mean_tf;
      pointTFToMsg(temp_mean_tf, temp_mean);
      free_means_.push_back(temp_mean);
    }
    for (auto cluster_it = occluded_clusters.begin(); cluster_it != occluded_clusters.end(); ++cluster_it){
      geometry_msgs::Point temp_mean;
      tf::Point temp_mean_tf;
      occluded_cluster_vectors_.push_back(getClusterInLaserFrame(*(cluster_it)));
      temp_mean = getMean(occluded_cluster_vectors_.back());
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
        // Init leg merge vector
        std::vector<int> leg_merge_ids;

        // Prediction
        tracked_objects_[i].state_mean =
          kalman_filter_.prediction(tracked_objects_[i].state_mean);
        tracked_objects_[i].state_var =
          kalman_filter_.prediction(tracked_objects_[i].state_var);

        // Find association by searching for closest measurement
        int id = -1;
        double distance = 10;
        std::vector<geometry_msgs::Point> all_means = free_means_;
        all_means.insert(all_means.end(), occluded_means_.begin(), occluded_means_.end());
        for (size_t j = 0; j < all_means.size(); j++){
          // Check distance (later try mahalanobis)
          double temp_distance =
                  pow(tracked_objects_[i].state_mean[0] - all_means[j].x, 2) +
                  pow(tracked_objects_[i].state_mean[1] - all_means[j].y, 2);
          // Check if current distance smaller
          if (temp_distance < association_radius_sq_ && temp_distance < distance){
            id = j;
            distance = temp_distance;
          }
          if (tracked_objects_[i].dynamic_or_static == "dynamic" && temp_distance < 1){
            // Check if cluster is in velocity dependent ellipse
            double a = 0.35;
            double v_norm = 0.8;
            double angle_temp = tan(tracked_objects_[i].state_mean[0] /
                                    tracked_objects_[i].state_mean[1]);
            double v = sqrt(pow(tracked_objects_[i].state_mean[0], 2) +
                            pow(tracked_objects_[i].state_mean[1], 2));
            double b = a * (1 + v / v_norm);
            a = a * (1 + v / (10 * v_norm));
            tf::Transform cluster_temp = xythetaToTF(all_means[j].x - tracked_objects_[i].state_mean[0],
                                                     all_means[j].y - tracked_objects_[i].state_mean[1],
                                                     0);
            tf::Transform rotation_tf = xythetaToTF(0, 0, angle_temp);
            cluster_temp = rotation_tf * cluster_temp;

            double value = pow(cluster_temp.getOrigin().getX(), 2) / pow(a, 2) +
                           pow(cluster_temp.getOrigin().getY(), 2) / pow(b, 2);

            if (value <= 1){
              leg_merge_ids.push_back(j);
            }
          }
        }

        // Check size of leg clusters to merge and merge then
        if (leg_merge_ids.size() >= 2){
          for (int m = leg_merge_ids.size() - 1; m > 0; m--){
            all_means[leg_merge_ids[0]].x += all_means[leg_merge_ids[m]].x;
            all_means[leg_merge_ids[0]].y += all_means[leg_merge_ids[m]].y;
            all_means.erase(all_means.begin() + leg_merge_ids[m]);
          }
          all_means[leg_merge_ids[0]].x /= leg_merge_ids.size();
          all_means[leg_merge_ids[0]].y /= leg_merge_ids.size();

          // currently assume that dynamic objects are free (not always true!)
          tracked_objects_[i].current_occlusion = "free";

          for (int m = leg_merge_ids.size() - 1; m > 0; m--){
            if (leg_merge_ids[m] >= free_means_.size()){
              auto it_delete = occluded_clusters.begin();
              std::advance(it_delete, leg_merge_ids[m] - free_means_.size());
              occluded_clusters.erase(it_delete);
              occluded_cluster_vectors_.erase(occluded_cluster_vectors_.begin() +
                                        leg_merge_ids[m] - free_means_.size());
              occluded_means_.erase(occluded_means_.begin() + leg_merge_ids[m]
                                   - free_means_.size());
            }
            else {
              auto it_delete = free_clusters.begin();
              std::advance(it_delete, leg_merge_ids[m]);
              free_clusters.erase(it_delete);
              free_cluster_vectors_.erase(free_cluster_vectors_.begin() +
                                          leg_merge_ids[m]);
              free_means_.erase(free_means_.begin() + leg_merge_ids[m]);
            }
          }

          id = leg_merge_ids[0];
        }

        // Check if found assignement, if yes update track
        if (id != -1){
          tracked_objects_[i].counter_not_seen = 0;
          // Update tracked object
          kalman_filter_.updateKalmanGain(tracked_objects_[i].state_var);

          // Get measurement
          Eigen::Vector2d temp_z_m;
          temp_z_m << all_means[id].x, all_means[id].y;

          if (tracked_objects_[i].dynamic_or_static != "static"){
            tracked_objects_[i].state_mean =
                kalman_filter_.update(tracked_objects_[i].state_mean, temp_z_m);
            tracked_objects_[i].state_var =
                kalman_filter_.update(tracked_objects_[i].state_var, temp_z_m);
          }

          // If object is in unknown space for longer time, assume it is static
          if (tracked_objects_[i].dynamic_or_static == "unknown"){
            if (tracked_objects_[i].unknown_since > unknown_since_threshold_){
              tracked_objects_[i].dynamic_or_static = "static";
              // Set velocity to zero
              tracked_objects_[i].state_mean[2] = 0;
              tracked_objects_[i].state_mean[3] = 0;
              tracked_objects_[i].state_mean[4] = 0;
              tracked_objects_[i].state_mean[5] = 0;
            }
            // Check if overall unknown time is higher than three times the threshold
            else if (tracked_objects_[i].unknown_since_ever > 3 * unknown_since_threshold_){
              tracked_objects_[i].dynamic_or_static = "static";
              // Set velocity to zero
              tracked_objects_[i].state_mean[2] = 0;
              tracked_objects_[i].state_mean[3] = 0;
              tracked_objects_[i].state_mean[4] = 0;
              tracked_objects_[i].state_mean[5] = 0;
            }
            else {
              tracked_objects_[i].dynamic_or_static = "unknown";
              tracked_objects_[i].unknown_since += 1;
              tracked_objects_[i].unknown_since_ever += 1;
            }
          }

          // If tracks are occluded
          if (id >= free_means_.size()){
            tracked_objects_[i].current_occlusion = "occluded";
            // Check if unknown, and if cluster is bigger than a possible person
            if (tracked_objects_[i].dynamic_or_static == "unknown"){
              auto tmp_clst = occluded_cluster_vectors_[id - free_means_.size()];
              if(pow(tmp_clst.front().x - tmp_clst.back().x, 2) +
                 pow(tmp_clst.front().y - tmp_clst.back().y, 2)
                 > static_size_threshold_sq_){
                tracked_objects_[i].dynamic_or_static = "static";
                // Set velocity to zero
                tracked_objects_[i].state_mean[2] = 0;
                tracked_objects_[i].state_mean[3] = 0;
                tracked_objects_[i].state_mean[4] = 0;
                tracked_objects_[i].state_mean[5] = 0;
              }
            }

            // Remove scan from static and dynamic scan
            auto it_delete = occluded_clusters.begin();
            std::advance(it_delete, id - free_means_.size());
            if (tracked_objects_[i].dynamic_or_static == "unknown"){
              for (auto cluster_it = (*it_delete).begin();
                   cluster_it != (*it_delete).end(); ++cluster_it){
                dynamic_scan_.ranges[cluster_it->first] = 0;
                static_scan_.ranges[cluster_it->first] = 0;
              }
            }
            else if (tracked_objects_[i].dynamic_or_static == "static"){
              for (auto cluster_it = (*it_delete).begin();
                   cluster_it != (*it_delete).end(); ++cluster_it){
                dynamic_scan_.ranges[cluster_it->first] = 0;
              }
              tracked_objects_[i].state_mean[2] = 0;
              tracked_objects_[i].state_mean[3] = 0;
              tracked_objects_[i].state_mean[4] = 0;
              tracked_objects_[i].state_mean[5] = 0;
            }
            else {} // "dynamic", remove at end

            // Delete cluster and mean
            occluded_clusters.erase(it_delete);
            occluded_cluster_vectors_.erase(occluded_cluster_vectors_.begin() +
                                            id - free_means_.size());
            occluded_means_.erase(occluded_means_.begin() + id - free_means_.size());
          }
          // If free clusters check if they are moving
          else {
            // If track was occluded before, set velocity to zero
            if (tracked_objects_[i].current_occlusion == "occluded"){
              tracked_objects_[i].current_occlusion = "free";
              tracked_objects_[i].state_mean[2] = 0;
              tracked_objects_[i].state_mean[3] = 0;
              tracked_objects_[i].state_mean[4] = 0;
              tracked_objects_[i].state_mean[5] = 0;
              tracked_objects_[i].unknown_since = 0;
              tracked_objects_[i].velocity_counter = 0;
            }

            if (tracked_objects_[i].dynamic_or_static == "unknown"){
              // Check if cluster is bigger than a possible person
              auto tmp_clst = free_cluster_vectors_[id];
              double cluster_size = pow(tmp_clst.front().x - tmp_clst.back().x, 2) +
                                    pow(tmp_clst.front().y - tmp_clst.back().y, 2);
              if(cluster_size > static_size_threshold_sq_){
                tracked_objects_[i].dynamic_or_static = "static";
                tracked_objects_[i].state_mean[2] = 0;
                tracked_objects_[i].state_mean[3] = 0;
                tracked_objects_[i].state_mean[4] = 0;
                tracked_objects_[i].state_mean[5] = 0;
              }
              // Check if track is moving
              else if (cluster_size < static_size_threshold_sq_ &&
                       tmp_clst.size() >= min_cluster_size_for_moving_ &&
                       tracked_objects_[i].dynamic_or_static == "unknown"){
                tracked_objects_[i].velocity_counter += 1;
                tracked_objects_[i].x_vel_sum += tracked_objects_[i].state_mean[2];
                tracked_objects_[i].y_vel_sum += tracked_objects_[i].state_mean[3];
                // Check if enough velocity counts available
                if (tracked_objects_[i].velocity_counter >= min_velocity_count_){
                  if (pow(tracked_objects_[i].x_vel_sum / tracked_objects_[i].velocity_counter, 2) +
                      pow(tracked_objects_[i].y_vel_sum / tracked_objects_[i].velocity_counter, 2)
                      > dynamic_vel_threshold_sq_){
                    tracked_objects_[i].dynamic_or_static = "dynamic";
                  }
                }
              }
            }

            // Remove scan from static and dynamic scan
            auto it_delete = free_clusters.begin();
            std::advance(it_delete, id);
            if (tracked_objects_[i].dynamic_or_static == "unknown"){
              for (auto cluster_it = (*it_delete).begin();
                   cluster_it != (*it_delete).end(); ++cluster_it){
                dynamic_scan_.ranges[cluster_it->first] = 0;
                static_scan_.ranges[cluster_it->first] = 0;
              }
            }
            else if (tracked_objects_[i].dynamic_or_static == "static"){
              for (auto cluster_it = (*it_delete).begin();
                   cluster_it != (*it_delete).end(); ++cluster_it){
                dynamic_scan_.ranges[cluster_it->first] = 0;
              }
              tracked_objects_[i].state_mean[2] = 0;
              tracked_objects_[i].state_mean[3] = 0;
              tracked_objects_[i].state_mean[4] = 0;
              tracked_objects_[i].state_mean[5] = 0;
            }
            else {} // "dynamic", remove at end

            // Delete cluster and mean
            free_clusters.erase(it_delete);
            free_cluster_vectors_.erase(free_cluster_vectors_.begin() + id);
            free_means_.erase(free_means_.begin() + id);
          }
        } // (id != -1)
        else {
          // Remove track if untracked for longer time
          if (tracked_objects_[i].counter_not_seen < not_seen_threshold_){
            tracked_objects_[i].counter_not_seen += 1;

            // Set acceleration to zero, as mostly lost tracks have high
            // acceleration because of occlusion.
            tracked_objects_[i].state_mean[4] = 0;
            tracked_objects_[i].state_mean[5] = 0;
          }
          else {
            tracked_objects_.erase(tracked_objects_.begin() + i);
            i -= 1;
          }
        }
      }
    }

    // Generate new tracks for all unassigned measurements
    std::vector<geometry_msgs::Point> all_means = free_means_;
    all_means.insert(all_means.end(), occluded_means_.begin(), occluded_means_.end());
    if (!all_means.empty()){
      for (size_t i = 0; i < all_means.size(); i++){
        TrackedObject temp_tracked_object(all_means[i].x, all_means[i].y);

        // Check if object is dynamic or static
        int map_id = positionToMapId(all_means[i].x, all_means[i].y, map_width_,
                                     map_height_, map_resolution_);

        auto it_delete = occluded_clusters.begin();
        if (i >= free_means_.size()){
          std::advance(it_delete, i - free_means_.size());
        }
        else{
          it_delete = free_clusters.begin();
          std::advance(it_delete, i);
        }

        // Save occlusion
        if (i >= free_means_.size()){
          temp_tracked_object.current_occlusion = "occluded";
        }
        else {
          temp_tracked_object.current_occlusion = "free";
        }

        // If object is in unknown space, mark as unknown
        if (map_msg_.data[map_id] == -1){
          temp_tracked_object.dynamic_or_static = "unknown";
          temp_tracked_object.unknown_since = 0;
          temp_tracked_object.unknown_since_ever = 0;
          // Remove from all scans as still unknown
          for (auto cluster_it = (*it_delete).begin();
               cluster_it != (*it_delete).end(); ++cluster_it){
            dynamic_scan_.ranges[cluster_it->first] = 0;
            static_scan_.ranges[cluster_it->first] = 0;
          }
        }
        else if (map_msg_.data[map_id] > wall_threshold_){
          // If object is in wall, assume it is static
          temp_tracked_object.dynamic_or_static = "static";
          // Remove from dynamic scan
          for (auto cluster_it = (*it_delete).begin();
               cluster_it != (*it_delete).end(); ++cluster_it){
            dynamic_scan_.ranges[cluster_it->first] = 0;
          }
        }
        else {
          temp_tracked_object.dynamic_or_static = "unknown";
          temp_tracked_object.unknown_since = 0;
          temp_tracked_object.unknown_since_ever = 0;
          // Remove from all scans as still unknown
          for (auto cluster_it = (*it_delete).begin();
               cluster_it != (*it_delete).end(); ++cluster_it){
            dynamic_scan_.ranges[cluster_it->first] = 0;
            static_scan_.ranges[cluster_it->first] = 0;
          }
        }

        tracked_objects_.push_back(temp_tracked_object);
      }
    }

    // Initialise Obstacle Array Msg for teb local planner
    costmap_converter::ObstacleArrayMsg dyn_obstacles;
    costmap_converter::ObstacleMsg obstacle;
    dyn_obstacles.header.frame_id = "map";
    dyn_obstacles.header.stamp = ros::Time::now();
    obstacle.header.frame_id = "map";
    obstacle.header.stamp = ros::Time::now();

    // Init obstacle values which stay constant
    tf::Quaternion quaternion;
    quaternion.setEuler(0, 0, 0);
    tf::quaternionTFToMsg(quaternion, obstacle.orientation);
    obstacle.velocities.twist.linear.z = 0;
    obstacle.velocities.twist.angular.x = 0;
    obstacle.velocities.twist.angular.y = 0;
    obstacle.velocities.twist.angular.z = 0;

    // Publish means of tracked objects as grid cells
    nav_msgs::GridCells moving_objects_msg;
    moving_objects_msg.header.frame_id = "map";
    moving_objects_msg.cell_width = 0.2;
    moving_objects_msg.cell_height = 0.2;
    nav_msgs::GridCells static_objects_msg;
    static_objects_msg.header.frame_id = "map";
    static_objects_msg.cell_width = 0.2;
    static_objects_msg.cell_height = 0.2;
    nav_msgs::GridCells unknown_objects_msg;
    unknown_objects_msg.header.frame_id = "map";
    unknown_objects_msg.cell_width = 0.2;
    unknown_objects_msg.cell_height = 0.2;
    for (size_t i = 0; i < tracked_objects_.size(); i++){
      geometry_msgs::Point temp_point;
      temp_point.x = tracked_objects_[i].state_mean[0];
      temp_point.y = tracked_objects_[i].state_mean[1];
      temp_point.z = 0;
      if (tracked_objects_[i].dynamic_or_static == "dynamic"){
        moving_objects_msg.cells.push_back(temp_point);

        // Save obstacle information and add to array
        obstacle.id = i; // id can change for the same object!
        geometry_msgs::Point32 point;
        point.x = temp_point.x;
        point.y = temp_point.y;
        point.z = temp_point.z;
        obstacle.polygon.points.push_back(point);

        obstacle.velocities.twist.linear.x = tracked_objects_[i].state_mean[2];
        obstacle.velocities.twist.linear.y = tracked_objects_[i].state_mean[3];

        dyn_obstacles.obstacles.push_back(obstacle);
        obstacle.polygon.points.pop_back();
      }
      else if (tracked_objects_[i].dynamic_or_static == "static"){
        static_objects_msg.cells.push_back(temp_point);
      }
      else {
        unknown_objects_msg.cells.push_back(temp_point);
      }
    }
    moving_objects_pub_.publish(moving_objects_msg);
    dyn_obstacles_pub_.publish(dyn_obstacles);
    static_objects_pub_.publish(static_objects_msg);
    unknown_objects_pub_.publish(unknown_objects_msg);

    // Update delta_t
    curr_node_stamp_ = dynamic_scan_.header.stamp;
    prev_delta_t_ = (curr_node_stamp_ - prev_node_stamp_).toSec();
    prev_node_stamp_ = curr_node_stamp_;

    // Remove dynamic scans from static scan
    for (int scan_i = 0; scan_i < dynamic_scan_.ranges.size(); scan_i++){
      if (dynamic_scan_.ranges[scan_i] != 0){
        static_scan_.ranges[scan_i] = 0;
      }
    }

    // Publish dynamic and static scan
    static_scan_pub_.publish(static_scan_);
    dynamic_scan_pub_.publish(dynamic_scan_);

    // Fill static_scan_queue_
    static_scan_queue_.push(static_scan_);

    // Free memory for avoiding memory leaking
    ld_free(current_ldp_);
  }
}

void StaticScanExtractor::mapCallback
    (const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
  map_msg_ = *map_msg;
  map_callback_initialised_ = true;
}


int main(int argc, char **argv){
  ros::init(argc, argv, "static_scan_extractor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  StaticScanExtractor extractor(nh, nh_);

  ros::spin();

  return 0;
}
