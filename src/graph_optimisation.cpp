/**
 *  graph_optimisation.cpp
 *
 *  Created on: 13.08.2018
 *      Author: Dario Mammolo
 */

#include <graph_optimisation.h>
#include "helper.cpp"

using namespace gtsam;

GraphOptimiser::GraphOptimiser(ros::NodeHandle nh, ros::NodeHandle nh_):
  nh_(nh), nh_private_(nh_)
{
  ROS_INFO("Started GraphOptimiser");
  initParams();
}

GraphOptimiser::~GraphOptimiser(){}

void GraphOptimiser::initParams(){
  // Get params which describe in which distance the node of the graph is built
  nh_private_.param<double>("node_dist_linear", node_dist_linear_, 0.10);
  nh_private_.param<double>("node_dist_angular", node_dist_angular_, 0.175);
  nh_private_.param<double>("loop_closing_radius", lc_radius_, 0.175);
  nh_private_.param<bool>("const_map_update_steps", const_map_update_steps_, false);
  nh_private_.param<int>("map_width", map_width_, 1000);
  nh_private_.param<int>("map_height", map_height_, 1000);
  nh_private_.param<float>("map_resolution", map_resolution_, 0.05);
  nh_private_.param<bool>("using_gazebo", using_gazebo_, false);
  nh_private_.param<std::string>("scan_callback_topic", scan_callback_topic_, "base_scan");

  // Initialise subscriber and publisher
  scan_sub_ = new message_filters::Subscriber
              <sensor_msgs::LaserScan>(nh_, scan_callback_topic_, 1);
  pose_sub_ = new message_filters::Subscriber
              <geometry_msgs::PoseWithCovarianceStamped>
              (nh_, "pose_with_covariance_stamped", 1);
  sync_ = new message_filters::TimeSynchronizer
          <sensor_msgs::LaserScan, geometry_msgs::PoseWithCovarianceStamped>
          (*scan_sub_, *pose_sub_, 1);
  sync_->registerCallback(boost::bind(&GraphOptimiser::scanMatcherCallback,
                                      this, _1, _2));

  path_pub_ = nh_.advertise<nav_msgs::Path>("graph_path", 1);
  action_path_pub_ = nh_.advertise<nav_msgs::Path>("action_graph", 1);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1);
  map_pub_for_static_scan_comb_ =
  nh_.advertise<nav_msgs::OccupancyGrid>("map_for_static_scan_extractor", 1);
  test_pose2D_pub_ = nh_.advertise<geometry_msgs::Pose2D>("test_pose2D", 1);
  test_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("test_pose", 1);

  // Initialise map service
  uncertainty_service_ = nh_.advertiseService("save_uncertainty_service",
                      &GraphOptimiser::saveUncertaintyMatServiceCallback, this);
  map_recalc_service_ = nh_.advertiseService("map_recalculation_service",
                      &GraphOptimiser::mapRecalculationServiceCallback, this);
  get_map_service_ = nh_.advertiseService("get_map_service",
                      &GraphOptimiser::getMapServiceCallback, this);
  utility_calc_service_ = nh_.advertiseService("utility_calc_service",
                      &GraphOptimiser::utilityCalcServiceCallback, this);
  current_pose_node_service_ = nh_.advertiseService("current_pose_node_service",
                      &GraphOptimiser::currentPoseNodeServiceCallback, this);
  get_ground_truth_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>
                                                    ("gazebo/get_model_state");

  // Initialize base to laser tf
  tf::StampedTransform base_to_laser_tf_;

  bool got_transform = false;
  while (!got_transform){
    if (using_gazebo_){
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

  sm_icp_params_.max_angular_correction_deg = 180.0;
  sm_icp_params_.max_linear_correction = 3 * lc_radius_;
  sm_icp_params_.max_iterations = 10;
  sm_icp_params_.epsilon_xy = 0.000001;
  sm_icp_params_.epsilon_theta = 0.000001;
  sm_icp_params_.max_correspondence_dist = 0.5;
  sm_icp_params_.use_corr_tricks = 1;
  sm_icp_params_.restart = 0;
  sm_icp_params_.restart_threshold_mean_error = 0.01;
  sm_icp_params_.restart_dt = 1.0;
  sm_icp_params_.restart_dtheta = 0.1;
  sm_icp_params_.outliers_maxPerc = 0.97;
  sm_icp_params_.outliers_adaptive_order = 0.98;
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
  sm_icp_params_.do_compute_covariance = 1;

  // Initialise noise on scan matching nodes
  average_scan_match_noise_ = noiseModel::Diagonal::Variances((Vector(3) << 0.0000001, 0.0000001, 0.00000001));

  // ISAM2
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam_ = ISAM2(parameters);

  // Init OccupancyGrid msg
  occupancy_grid_msg_.header.frame_id = "map";
  occupancy_grid_msg_.info.resolution = map_resolution_;
  occupancy_grid_msg_.info.width = map_width_;
  occupancy_grid_msg_.info.height = map_height_;

  // Origin
  geometry_msgs::Pose origin_pose;
  origin_pose = xythetaToPose(-(map_width_ * map_resolution_ / 2),
                              -(map_height_ * map_resolution_ / 2),
                              0);
  occupancy_grid_msg_.info.origin = origin_pose;

  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      occupancy_grid_msg_.data.push_back(-1);
    }
  }

  // Init max allowed scan range
  max_range_allowed_ = 30;

  // Init log log_odds_array_
  log_odds_array_ = Eigen::MatrixXf(map_width_, map_height_);
  l_0_ = log(0.5 / 0.5);

  // Asuming sensor model: P(z=1|m=1)=0.8 because can eg. go through glass
  //                       P(z=0|m=0)=0.99
  p_occ_ = 0.998;       // P(m=1|z=1)
  p_free_ = 0.168;      // P(m=1|z=0)
  l_occ_ = log(p_occ_ / (1.0 - p_occ_));
  l_free_ = log(p_free_ / (1.0 - p_free_));

  // Initialise other parameters
  dist_linear_sq_ = node_dist_linear_ * node_dist_linear_;
  first_scan_pose_ = true;
  scan_callback_initialized_ = false;
  new_node_ = false;
  first_map_calculated_ = false;
  node_counter_ = 0;

  feature_labels_.push_back(PointMatcher<float>::DataPoints::Label("x", 1));
  feature_labels_.push_back(PointMatcher<float>::DataPoints::Label("y", 1));
  feature_labels_.push_back(PointMatcher<float>::DataPoints::Label("pad", 1));

  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string config_path = package_path + "/config/icp_cfg.yaml";

  std::ifstream config_file(config_path.c_str());

  icp_.loadFromYaml(config_file);

  // Publish map in other thread at low rate (only for rviz)
  boost::thread map_pub_thread(&GraphOptimiser::pubMap, this);
}

void GraphOptimiser::laserScanToMatrix(sensor_msgs::LaserScan& scan_msg, Eigen::MatrixXf& matrix){
  Eigen::MatrixXf temp_mat(3, scan_ranges_size_);

  for (unsigned int i = 0; i < scan_ranges_size_; i++){
    temp_mat(0, i) = scan_msg.ranges[i] * cos(scan_msg.angle_min + i * scan_angle_increment_);
    temp_mat(1, i) = scan_msg.ranges[i] * sin(scan_msg.angle_min + i * scan_angle_increment_);
    temp_mat(2, i) = 1;
  }

  matrix = temp_mat;
}

void GraphOptimiser::laserScanToLDP(sensor_msgs::LaserScan& scan_msg, LDP& ldp){
  unsigned int n = scan_ranges_size_;
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++){
    double r = scan_msg.ranges[i];

    if (r > scan_range_min_ && r < max_range_allowed_){
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

void GraphOptimiser::getSubsetOfMap(nav_msgs::Path& action_path,
                                    std::map<int, int>& subset){
  //
  std::map<int,int>::iterator finder;
  int node_size = action_path.poses.size();
  for (int i = 0; i < node_size; i++){
    // Get pose of laser
    tf::Pose robot_tf;
    tf::poseMsgToTF(action_path.poses[i].pose, robot_tf);
    tf::Transform laser_tf;
    laser_tf = robot_tf * base_to_laser_;
    double laser_x = laser_tf.getOrigin().getX();
    double laser_y = laser_tf.getOrigin().getY();

    std::vector<int> laser_idx = positionToMapIndex(laser_x, laser_y,
                                      map_width_, map_height_, map_resolution_);
    int x0 = laser_idx[0];
    int y0 = laser_idx[1];

    // Bresenham
    double theta = 0;
    for (int j = 0; j < scan_ranges_size_; j++){
      bool wall_reached = false;
      int dx = cos(theta);
      int dy = sin(theta);
      int xstep = 1;
      int ystep = 1;

      // Check if dx, dy are negative (direction changes)
      if (dx < 0) {dx = -dx; xstep = -1;};
      if (dy < 0) {dy = -dy; ystep = -1;};

      // Calculate variable for performance improvement
      int twodx = 2 * dx;
      int twody = 2 * dy;

      // Check if gradient < 1
      if (dx > dy){
        int fraction_increment = 2 * dy;
        int fraction = 2 * dy - dx;
        int x = x0 + xstep;
        int y = y0;
        while (!wall_reached){
          fraction += fraction_increment;
          if (fraction >= 0){
            y += ystep;
            fraction -= twodx;
          }
          int temp_id = mapIndexToId(x, y, map_width_);
          // Check if next cell is not wall and not unknown
          int temp_prob = int(occupancy_grid_msg_.data[temp_id]);
          if (temp_prob < 90 && temp_prob != -1){
            // Check if already in subset
            finder = subset.find(temp_id);
            if (finder != subset.end()){
              finder->second = i;
            }
            else {
              subset.insert(std::make_pair(temp_id, i));
            }
          }
          else if (temp_prob >= 90){
            // Check if already in subset
            finder = subset.find(temp_id);
            if (finder != subset.end()){
              finder->second = i;
            }
            else {
              subset.insert(std::make_pair(temp_id, i));
            }
            wall_reached = true;
          }
          // if unknown
          else {
            wall_reached = true;
          }
          x += xstep;
        }
      }
      else {
        int fraction_increment = 2 * dx;
        int fraction = 2 * dx - dy;
        int x = x0;
        int y = y0 + ystep;
        while (!wall_reached){
          fraction += fraction_increment;
          if (fraction >= 0){
            x += xstep;
            fraction -= twody;
          }
          int temp_id = mapIndexToId(x, y, map_width_);
          // Check if next cell is not a wall or unkown
          int temp_prob = int(occupancy_grid_msg_.data[temp_id]);
          if (temp_prob < 90 && temp_prob != -1){
            // Check if already in subset
            finder = subset.find(temp_id);
            if (finder != subset.end()){
              finder->second = i;
            }
            else {
              subset.insert(std::make_pair(temp_id, i));
            }
          }
          else if (temp_prob >= 90){
            // Check if already in subset
            finder = subset.find(temp_id);
            if (finder != subset.end()){
              finder->second = i;
            }
            else {
              subset.insert(std::make_pair(temp_id, i));
            }
            wall_reached = true;
          }
          // if unknown
          else {
            wall_reached = true;
          }
          y += ystep;
        }
      }
      theta += scan_angle_increment_;
    }
  }
}

void GraphOptimiser::updateLogOdsWithBresenham(int x0, int y0, int x1, int y1,
                               std::vector<int> end_point_index_m, bool update){
  // Bresenham's line algorithm (Get indexes between robot pose and scans)
  // starting from "https://github.com/lama-imr/lama_utilities/blob/indigo-devel/map_ray_caster/src/map_ray_caster.cpp"
  // "https://csustan.csustan.edu/~tom/Lecture-Notes/Graphics/Bresenham-Line/Bresenham-Line.pdf"

  int dx = x1 - x0;
  int dy = y1 - y0;
  int xstep = 1;
  int ystep = 1;

  // Check if dx, dy are negative (direction changes)
  if (dx < 0) {dx = -dx; xstep = -1;};
  if (dy < 0) {dy = -dy; ystep = -1;};

  // Calculate variable for performance improvement
  int twodx = 2 * dx;
  int twody = 2 * dy;

  // Check if gradient < 1
  if (dx > dy){
    int fraction_increment = 2 * dy;
    int fraction = 2 * dy - dx;
    int x = x0 + xstep;
    int y = y0;
    for (; x != x1; x += xstep){
      fraction += fraction_increment;
      if (fraction >= 0){
        y += ystep;
        fraction -= twodx;
      }
      if (x != end_point_index_m[0] || y != end_point_index_m[1]){
        log_odds_array_(x, y) += l_free_ - l_0_;
        if (update){
          unsigned int temp_id = mapIndexToId(x, y, map_width_);
          occupancy_grid_msg_.data[temp_id] = 100 * (1.0 - 1.0 / (1.0 +
                                                  exp(log_odds_array_(x, y))));
        }
      }
    }
  }
  else {
    int fraction_increment = 2 * dx;
    int fraction = 2 * dx - dy;
    int x = x0;
    int y = y0 + ystep;
    for (; y != y1; y += ystep){
      fraction += fraction_increment;
      if (fraction >= 0){
        x += xstep;
        fraction -= twody;
      }
      if (x != end_point_index_m[0] || y != end_point_index_m[1]){
        log_odds_array_(x, y) += l_free_ - l_0_;
        if (update){
          unsigned int temp_id = mapIndexToId(x, y, map_width_);
          occupancy_grid_msg_.data[temp_id] = 100 * (1.0 - 1.0 / (1.0 +
                                                  exp(log_odds_array_(x, y))));
        }
      }
    }
  }
}

void GraphOptimiser::drawMap(gtsam::Values& pose_estimates,
                             std::vector<LDP>& keyframe_ldp_vec){
  // Init log_odds_array_
  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      log_odds_array_(i, j) = l_0_;
    }
  }

  // Loop over each keyframe
  for (unsigned int i = 0; i < keyframe_ldp_vec.size(); i++){
    Pose2 pose2_estimate = *dynamic_cast<const Pose2*>(&pose_estimates.at(i));
    tf::Transform map_to_laser_tf;
    map_to_laser_tf = xythetaToTF(pose2_estimate.x(),
                                  pose2_estimate.y(),
                                  pose2_estimate.theta()) * base_to_laser_;

    std::vector<int> laser_pose_index;

    double laser_x = map_to_laser_tf.getOrigin().getX();
    double laser_y = map_to_laser_tf.getOrigin().getY();
    laser_pose_index = positionToMapIndex(laser_x,
                                          laser_y,
                                          map_width_,
                                          map_height_,
                                          map_resolution_);

    // Save index in shorter variable for later use
    int x0 = laser_pose_index[0];
    int y0 = laser_pose_index[1];

    // Loop over each ray of the scan
    for (int j = 0; j < keyframe_ldp_vec[i]->nrays; j++){
      double reading = keyframe_ldp_vec[i]->readings[j];

      if (reading == 0) {
        continue;
      }
      else if (reading == -1){
        // We can assume that there exist never a reading < 0.1
        reading = max_range_allowed_;
      }

      double angle = keyframe_ldp_vec[i]->theta[j];
      double x_end = reading * cos(angle);
      double y_end = reading * sin(angle);
      tf::Transform endpoint;
      endpoint = map_to_laser_tf * xythetaToTF(x_end, y_end, angle);

      double endpoint_x = endpoint.getOrigin().getX();
      double endpoint_y = endpoint.getOrigin().getY();

      std::vector<int> end_point_index;
      end_point_index = positionToMapIndex(endpoint_x,
                                           endpoint_y,
                                           map_width_,
                                           map_height_,
                                           map_resolution_);

      // Save index in shorter variable for later use
      int x1 = end_point_index[0];
      int y1 = end_point_index[1];

      // Update log odds of scan points
      if (reading == max_range_allowed_){
        log_odds_array_(x1, y1) += l_free_ - l_0_;
      }
      else {
        log_odds_array_(x1, y1) += l_occ_ - l_0_;
      }

      // Check if +/- 0.5*resolution is in a new cell
      double xdiff = endpoint_x - laser_x;
      double ydiff = endpoint_y - laser_y;

      // Calculate delta_x/y with similar triangles
      double delta_x = 0.5 * map_resolution_ / reading * xdiff;
      double delta_y = 0.5 * map_resolution_ / reading * ydiff;

      std::vector<int> end_point_index_p;
      end_point_index_p = positionToMapIndex(endpoint_x + delta_x,
                                             endpoint_y + delta_y,
                                             map_width_,
                                             map_height_,
                                             map_resolution_);

      std::vector<int> end_point_index_m;
      end_point_index_m = positionToMapIndex(endpoint_x - delta_x,
                                             endpoint_y - delta_y,
                                             map_width_,
                                             map_height_,
                                             map_resolution_);

      if (end_point_index_p[0] != x1 || end_point_index_p[1] != y1){
        if (reading == scan_range_max_){
          log_odds_array_(end_point_index_p[0], end_point_index_p[1]) += l_free_ - l_0_;
        }
        else {
          log_odds_array_(end_point_index_p[0], end_point_index_p[1]) += l_occ_ - l_0_;
        }
      }
      if (end_point_index_m[0] != x1 || end_point_index_m[1] != y1){
        if (reading == scan_range_max_){
          log_odds_array_(end_point_index_m[0], end_point_index_m[1]) += l_free_ - l_0_;
        }
        else {
          log_odds_array_(end_point_index_m[0], end_point_index_m[1]) += l_occ_ - l_0_;
        }
      }

      updateLogOdsWithBresenham(x0, y0, x1, y1, end_point_index_m, false);
    }
  }

  // Update occupancy_grid_msg_
  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      if (log_odds_array_(i, j) == l_0_){
        occupancy_grid_msg_.data[mapIndexToId(i, j, map_width_)] = -1;
      }else {
        occupancy_grid_msg_.data[mapIndexToId(i, j, map_width_)] =
                        100 * (1.0 - 1.0 / (1.0 + exp(log_odds_array_(i, j))));
      }
    }
  }
}

void GraphOptimiser::updateMap(gtsam::Values& pose_estimates,
                               std::vector<LDP>& keyframe_ldp_vec){
  int i = keyframe_ldp_vec.size() - 1;
  Pose2 pose2_estimate = *dynamic_cast<const Pose2*>(&pose_estimates.at(i));
  tf::Transform map_to_laser_tf;
  map_to_laser_tf = xythetaToTF(pose2_estimate.x(),
                                pose2_estimate.y(),
                                pose2_estimate.theta()) * base_to_laser_;

  std::vector<int> laser_pose_index;

  double laser_x = map_to_laser_tf.getOrigin().getX();
  double laser_y = map_to_laser_tf.getOrigin().getY();
  laser_pose_index = positionToMapIndex(laser_x,
                                        laser_y,
                                        map_width_,
                                        map_height_,
                                        map_resolution_);

  // Save index in shorter variable for later use
  int x0 = laser_pose_index[0];
  int y0 = laser_pose_index[1];

  // Loop over each ray of the scan
  for (int j = 0; j < keyframe_ldp_vec[i]->nrays; j++){
    double reading = keyframe_ldp_vec[i]->readings[j];

    if (reading == 0) {
      continue;
    }
    else if (reading == -1){
      // We can assume that there exist never a reading < 0.1
      reading = max_range_allowed_;
    }

    double angle = keyframe_ldp_vec[i]->theta[j];
    double x_end = reading * cos(angle);
    double y_end = reading * sin(angle);
    tf::Transform endpoint;
    endpoint = map_to_laser_tf * xythetaToTF(x_end, y_end, angle);

    double endpoint_x = endpoint.getOrigin().getX();
    double endpoint_y = endpoint.getOrigin().getY();

    std::vector<int> end_point_index;
    end_point_index = positionToMapIndex(endpoint_x,
                                         endpoint_y,
                                         map_width_,
                                         map_height_,
                                         map_resolution_);

    // Save index in shorter variable for later use
    int x1 = end_point_index[0];
    int y1 = end_point_index[1];

    // Update log odds of scan points
    if (reading == max_range_allowed_){
      log_odds_array_(x1, y1) += l_free_ - l_0_;
    }
    else {
      log_odds_array_(x1, y1) += l_occ_ - l_0_;
    }
    unsigned int temp_id = mapIndexToId(x1, y1, map_width_);
    occupancy_grid_msg_.data[temp_id] = 100 * (1.0 - 1.0 / (1.0 + exp(log_odds_array_(x1, y1))));

    // Check if +/- 0.5*resolution is in a new cell
    double xdiff = endpoint_x - laser_x;
    double ydiff = endpoint_y - laser_y;

    // Calculate delta_x/y with similar triangles
    double delta_x = 0.5 * map_resolution_ / reading * xdiff;
    double delta_y = 0.5 * map_resolution_ / reading * ydiff;

    std::vector<int> end_point_index_p;
    end_point_index_p = positionToMapIndex(endpoint_x + delta_x,
                                           endpoint_y + delta_y,
                                           map_width_,
                                           map_height_,
                                           map_resolution_);

    std::vector<int> end_point_index_m;
    end_point_index_m = positionToMapIndex(endpoint_x - delta_x,
                                           endpoint_y - delta_y,
                                           map_width_,
                                           map_height_,
                                           map_resolution_);

    if (end_point_index_p[0] != x1 || end_point_index_p[1] != y1){
      if (reading == scan_range_max_){
        log_odds_array_(end_point_index_p[0], end_point_index_p[1]) += l_free_ - l_0_;
      }
      else {
        log_odds_array_(end_point_index_p[0], end_point_index_p[1]) += l_occ_ - l_0_;
      }
      unsigned int temp_id =
          mapIndexToId(end_point_index_p[0], end_point_index_p[1], map_width_);
      occupancy_grid_msg_.data[temp_id] = 100 * (1.0 - 1.0 / (1.0 +
          exp(log_odds_array_(end_point_index_p[0], end_point_index_p[1]))));
    }
    if (end_point_index_m[0] != x1 || end_point_index_m[1] != y1){
      if (reading == scan_range_max_){
        log_odds_array_(end_point_index_m[0], end_point_index_m[1]) += l_free_ - l_0_;
      }
      else {
        log_odds_array_(end_point_index_m[0], end_point_index_m[1]) += l_occ_ - l_0_;
      }
      unsigned int temp_id =
          mapIndexToId(end_point_index_m[0], end_point_index_m[1], map_width_);
      occupancy_grid_msg_.data[temp_id] = 100 * (1.0 - 1.0 / (1.0 +
          exp(log_odds_array_(end_point_index_m[0], end_point_index_m[1]))));
    }

    updateLogOdsWithBresenham(x0, y0, x1, y1, end_point_index_m, true);
  }
}

bool GraphOptimiser::saveUncertaintyMatServiceCallback(
  crowdbot_active_slam::service_call::Request &request,
  crowdbot_active_slam::service_call::Response &response){
  // Save current time
  std::time_t now = std::time(0);
  char char_time[100];
  std::strftime(char_time, sizeof(char_time), "_%Y_%m_%d_%H_%M_%S", std::localtime(&now));

  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string save_path = package_path + "/test_results/uncertainty_matrices" +
                          char_time + ".txt";

  Pose2 current_pose;
  Pose2 previous_pose;
  double path_length = 0.0;
  // Save uncertainty
  std::ofstream map_file(save_path.c_str());
  if (map_file.is_open()){
    for (unsigned int i = 0; i < uncertainty_matrices_path_.size(); i++){
      if (i == 0){
        map_file << path_length << " " << uncertainty_matrices_path_[i] << std::endl;
        previous_pose = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));
      }
      else {
        current_pose = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));
        path_length += current_pose.between(previous_pose).t().norm();
        map_file << path_length << " " << uncertainty_matrices_path_[i] << std::endl;
        previous_pose = current_pose;
      }
    }
    map_file.close();
  }
  else{
    ROS_INFO("Could not open uncertainty_matrices.txt!");
  }

  return true;
}

bool GraphOptimiser::mapRecalculationServiceCallback(
  crowdbot_active_slam::service_call::Request &request,
  crowdbot_active_slam::service_call::Response &response){

  // Draw the whole map
  drawMap(pose_estimates_, keyframe_ldp_vec_);
  return true;
  }

bool GraphOptimiser::utilityCalcServiceCallback(
  crowdbot_active_slam::utility_calc::Request &request,
  crowdbot_active_slam::utility_calc::Response &response){

  // ISAM2
  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  ISAM2 isam(parameters);

  Pose2 current_estimate;
  int size = pose_estimates_.size();

  // Cast Pose2 from Value
  current_estimate = *dynamic_cast<const Pose2*>(&pose_estimates_.at(size - 1));

  // Get covariance from current position
  noiseModel::Gaussian::shared_ptr current_marginal_noise;
  if (node_counter_ == 1){
    current_marginal_noise = average_scan_match_noise_;
  }
  else {
    current_marginal_noise =
        noiseModel::Gaussian::Covariance(isam_.marginalCovariance(size - 1));
  }

  // Build action graph
  NonlinearFactorGraph action_graph;
  Values action_estimates;
  action_graph.add(PriorFactor<Pose2>(0, current_estimate, current_marginal_noise));
  action_estimates.insert(0, current_estimate);

  Pose2 prev_pose = current_estimate;
  int node_counter = 1;
  int lc_counter = 0;
  unsigned char lc = 'l';
  std::map<int, int> map_of_lc;
  int prev_i = 0;
  double prev_angle = current_estimate.theta();
  double path_length = 0;

  for (unsigned int i = 0; i < request.plan.poses.size(); i++){
    // Check if distance/angle diff is bigger than threshold as done in SLAM algo below
    double x_diff = request.plan.poses[i].pose.position.x -
                    request.plan.poses[prev_i].pose.position.x;
    double y_diff = request.plan.poses[i].pose.position.y -
                    request.plan.poses[prev_i].pose.position.y;
    double diff_dist_linear_sq = x_diff * x_diff + y_diff * y_diff;

    double angle = xyDiffToYaw(x_diff, y_diff);
    double angle_diff = angle - prev_angle;

    if ((diff_dist_linear_sq > dist_linear_sq_) or
       (std::abs(angle_diff) > node_dist_angular_)){
      //
      Pose2 next_pose(request.plan.poses[i].pose.position.x,
                      request.plan.poses[i].pose.position.y,
                      angle);
      Pose2 next_mean = prev_pose.between(next_pose);

      // Update path length
      if (request.exploration_type == "utility_normalized"){
        path_length += next_mean.t().norm();
      }

      action_graph.add(BetweenFactor<Pose2>(node_counter - 1, node_counter,
        next_mean, average_scan_match_noise_));
      action_estimates.insert(node_counter, next_pose);

      // Check for loop closings
      double x_high = next_pose.x() + lc_radius_;
      double x_low = next_pose.x() - lc_radius_;
      double y_high = next_pose.y() + lc_radius_;
      double y_low = next_pose.y() - lc_radius_;
      double lc_radius_squared = lc_radius_ * lc_radius_;

      for (unsigned int j = 0; j < pose_estimates_.size() - 1; j++){
        Pose2 tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(j));

        // Check if tmp_pose2 is in square region around current pose estimate
        if (tmp_pose2.x() <= x_high && tmp_pose2.x() >= x_low &&
            tmp_pose2.y() <= y_high && tmp_pose2.y() >= y_low){
          double x_diff_lc = next_pose.x() - tmp_pose2.x();
          double y_diff_lc = next_pose.y() - tmp_pose2.y();

          // Check if tmp_pose2 is in circle around current pose estimate
          if (x_diff_lc * x_diff_lc + y_diff_lc * y_diff_lc <= lc_radius_squared){

            // Get Pose tf between current and new Pose as first guess for icp
            Pose2 diff_pose2 = next_pose.between(tmp_pose2);

            current_marginal_noise =
                noiseModel::Gaussian::Covariance(isam_.marginalCovariance(j));

            if (map_of_lc.find(j) != map_of_lc.end()){
              int lc_index = map_of_lc.find(j)->second;
              action_graph.add(BetweenFactor<Pose2>(node_counter, Symbol(lc, lc_index),
                                              diff_pose2, average_scan_match_noise_));
            }
            else {
              action_graph.add(PriorFactor<Pose2>(Symbol(lc, lc_counter),
                                              tmp_pose2, current_marginal_noise));
              action_graph.add(BetweenFactor<Pose2>(node_counter, Symbol(lc, lc_counter),
                                              diff_pose2, average_scan_match_noise_));
              action_estimates.insert(Symbol(lc, lc_counter), tmp_pose2);

              map_of_lc.insert(std::make_pair(j, lc_counter));
              lc_counter += 1;
            }
          }
        }
      }
      //
      //ROS_INFO("Optimisation started!");
      isam.update(action_graph, action_estimates);
      isam.update();

      action_graph.resize(0);
      action_estimates.clear();

      node_counter += 1;
      prev_angle = angle;
      prev_pose = next_pose;
      prev_i = i;
    }
  }

  action_estimates = isam.calculateEstimate();

  // Create a path msg of the graph node estimates
  nav_msgs::Path action_path;
  action_path.header.frame_id = "/map";
  Pose2 tmp_pose2;

  // Iterate over all node estimates
  int node_size = action_estimates.size() - lc_counter;
  for (int i = 0; i < node_size; i++){
    // Cast Pose2 from Value
    tmp_pose2 = *dynamic_cast<const Pose2*>(&action_estimates.at(i));

    // Create PoseStamped variables from Pose 2 and add to path
    action_path.poses.push_back(pose2ToPoseStamped(tmp_pose2));
  }

  // Publish the graph
  action_path_pub_.publish(action_path);

  // Calculate alpha for each node
  std::vector<double> alpha;
  double sigma_temp;

  for (int i = 0; i < node_size; i++){
    // D-optimality
    Eigen::VectorXcd eivals = isam.marginalCovariance(i).eigenvalues();
    double sum_of_logs = log(eivals[0].real()) +
                         log(eivals[1].real()) +
                         log(eivals[2].real());
    sigma_temp = exp(1.0 / 3.0 * sum_of_logs);
    alpha.push_back(1.0 + 1.0 / sigma_temp);
  }

  std::map<int, int> subset;
  getSubsetOfMap(action_path, subset);

  // Calculate utility
  double utility = 0;
  for (std::map<int, int>::iterator it = subset.begin(); it != subset.end(); ++it){
    // Get probability
    int p_percent = int(occupancy_grid_msg_.data[it->first]);
    // Check if cell is unkonwn
    if (p_percent == -1) p_percent = 50;
    // Check if not 0 or 1 to avoid nan's
    if (p_percent != 0 && p_percent != 100){
      double p = double(p_percent) / 100.0;
      if (alpha[it->second] > 1000){
        utility += -(p * log2(p) + (1 - p) * log2(1 - p)) + log2(std::max(p, 1 - p));
      }
      else{
        utility += -(p * log2(p) + (1 - p) * log2(1 - p)) -
        1.0 / (1.0 - alpha[it->second]) * log2(pow(p, alpha[it->second]) + pow(1 - p, alpha[it->second]));
      }

      // std::cout << "aplha: " << it->second << std::endl;
      // std::cout << "renyi: " << log2(pow(p, it->second) + pow(1 - p, it->second)) << std::endl;
      // std::cout << "utility: " << utility << std::endl;
    }
  }

  // normalize utility by path path_length
  if (request.exploration_type == "utility_normalized"){
    utility = utility / path_length;
  }
  response.utility = utility;
  return true;
}

void GraphOptimiser::scanMatcherCallback(
            const sensor_msgs::LaserScan::ConstPtr& scan_msg,
            const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
  // Laser scan callback
  latest_scan_msg_ = *scan_msg;
  if (!scan_callback_initialized_){
    scan_ranges_size_ = latest_scan_msg_.ranges.size();
    scan_angle_increment_ = latest_scan_msg_.angle_increment;
    scan_range_min_ = latest_scan_msg_.range_min;
    scan_range_max_ = latest_scan_msg_.range_max;
    scan_callback_initialized_ = true;
  }
  // Check if scan callback initialized (just to be sure)
  if (!scan_callback_initialized_) return;

  // Check if same stamp  (just to be sure)
  if (latest_scan_msg_.header.stamp != (*pose_msg).header.stamp){
   ROS_WARN("Different scan and pose stamps! Scans are skipped!");
   return;
  }

  // Get newest transform from odom to base_link for later use
  tf::StampedTransform odom_stamped_transform;
  if (using_gazebo_){
    odom_listener_.waitForTransform("/base_link", "/odom",
                                    ros::Time(0), ros::Duration(1.0));
    odom_listener_.lookupTransform("/base_link", "/odom", ros::Time(0),
                                   odom_stamped_transform);
  }
  else{
    odom_listener_.waitForTransform("/base_footprint", "/odom",
                                    ros::Time(0), ros::Duration(1.0));
    odom_listener_.lookupTransform("/base_footprint", "/odom", ros::Time(0),
                                   odom_stamped_transform);
  }

  tf::Transform odom_transform;
  odom_transform = static_cast<tf::Transform>(odom_stamped_transform);

  // Save msg as Pose2
  tf::Quaternion temp_orientation;
  quaternionMsgToTF((*pose_msg).pose.pose.orientation, temp_orientation);
  current_pose2_odom_ = Pose2((*pose_msg).pose.pose.position.x,
                              (*pose_msg).pose.pose.position.y,
                              tf::getYaw(temp_orientation));

  noiseModel::Diagonal::shared_ptr temp_scan_match_noise =
         noiseModel::Diagonal::Variances(
                               (Vector(3) << (*pose_msg).pose.covariance[0],
                                             (*pose_msg).pose.covariance[7],
                                             (*pose_msg).pose.covariance[35]));

  // Check if it is the first scan
  if (first_scan_pose_){
    // if robot is in gazebo, take gazebo init pose for map
    if (using_gazebo_) {
      gazebo_msgs::GetModelState ground_truth_msg;
      ground_truth_msg.request.model_name = "pioneer";
      get_ground_truth_client_.call(ground_truth_msg);
      tf::Quaternion ground_truth_orientation;
      quaternionMsgToTF(ground_truth_msg.response.pose.orientation, ground_truth_orientation);
      init_pose2_map_ = Pose2(ground_truth_msg.response.pose.position.x,
                          ground_truth_msg.response.pose.position.y,
                          tf::getYaw(ground_truth_orientation));
    }
    else {
      init_pose2_map_ = Pose2(0.0, 0.0, 0.0);
    }

    // Initialise map to odom tf
    tf::Transform init_tf;
    init_tf.setOrigin(tf::Vector3(init_pose2_map_.x(), init_pose2_map_.y(), 0.0));
    tf::Quaternion init_q;
    init_q.setRPY(0.0, 0.0, init_pose2_map_.theta());
    init_tf.setRotation(init_q);
    map_to_odom_tf_ = init_tf;
    map_br_.sendTransform(tf::StampedTransform(map_to_odom_tf_, ros::Time::now(),
                          "map", "odom"));

    // Create graph with first node
    pose_estimates_.insert(node_counter_, init_pose2_map_);
    new_estimates_ = pose_estimates_;
    noiseModel::Diagonal::shared_ptr prior_noise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.001, 0.001, 0.0001));
    graph_.add(PriorFactor<Pose2>(node_counter_, init_pose2_map_, prior_noise));

    // Save scan as LDP
    laserScanToLDP(latest_scan_msg_, current_ldp_);
    keyframe_ldp_vec_.push_back(current_ldp_);

    // Init some sm icp params from laser scan msg
    sm_icp_params_.min_reading = latest_scan_msg_.range_min;
    sm_icp_params_.max_reading = latest_scan_msg_.range_max;

    Matrix initial_covariance(3, 3);
    initial_covariance << 0.00001, 0.0, 0.0,
                          0.0, 0.00001, 0.0,
                          0.0, 0.0, 0.0000001;
    uncertainty_matrices_path_.push_back(initial_covariance);

    // Update variables
    node_counter_ += 1;
    first_scan_pose_ = false;
    last_pose2_map_ = init_pose2_map_;
    latest_pose_estimate_ = pose2ToPoseStamped(last_pose2_map_);
    latest_pose_estimate_.header.stamp = latest_scan_msg_.header.stamp;
    prev_pose2_odom_ = current_pose2_odom_;
  }
  else {
    // Calculate some variables for decision if to add a new node
    double x_diff = current_pose2_odom_.x() - prev_pose2_odom_.x();
    double y_diff = current_pose2_odom_.y() - prev_pose2_odom_.y();
    double diff_dist_linear_sq = x_diff * x_diff + y_diff * y_diff;
    double angle_diff = current_pose2_odom_.theta() - prev_pose2_odom_.theta();

    // Add a new node if robot moved far enough
    if ((diff_dist_linear_sq > dist_linear_sq_) or
       (std::abs(angle_diff) > node_dist_angular_)){
      new_node_ = true;

      // Save scan as LDP
      laserScanToLDP(latest_scan_msg_, current_ldp_);
      current_ldp_->estimate[0] = 0.0;
      current_ldp_->estimate[1] = 0.0;
      current_ldp_->estimate[2] = 0.0;
      keyframe_ldp_vec_.push_back(current_ldp_);

      // Add new node to graph
      Pose2 next_mean = prev_pose2_odom_.between(current_pose2_odom_);
      Pose2 current_pose2_map = last_pose2_map_ * next_mean;
      pose_estimates_.insert(node_counter_, current_pose2_map);
      new_estimates_.insert(node_counter_, current_pose2_map);
      graph_.add(BetweenFactor<Pose2>(node_counter_ - 1, node_counter_,
                 next_mean, temp_scan_match_noise));

      // look for loop closing factors
      double x_high = current_pose2_map.x() + lc_radius_;
      double x_low = current_pose2_map.x() - lc_radius_;
      double y_high = current_pose2_map.y() + lc_radius_;
      double y_low = current_pose2_map.y() - lc_radius_;
      double lc_radius_squared = lc_radius_ * lc_radius_;

      for (int i = 0; i < node_counter_ - 1; i++){
        Pose2 tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));

        // Check if tmp_pose2 is in square region around current pose estimate
        if (tmp_pose2.x() <= x_high && tmp_pose2.x() >= x_low &&
            tmp_pose2.y() <= y_high && tmp_pose2.y() >= y_low){
          double x_diff = current_pose2_map.x() - tmp_pose2.x();
          double y_diff = current_pose2_map.y() - tmp_pose2.y();

          // Check if tmp_pose2 is in circle around current pose estimate
          if (x_diff * x_diff + y_diff * y_diff <= lc_radius_squared){
            // Save laser scans for mapping
            sm_icp_params_.laser_ref = current_ldp_;
            sm_icp_params_.laser_sens = keyframe_ldp_vec_[i];

            // Get Pose tf between current and new Pose as first guess for icp
            Pose2 diff_pose2 = current_pose2_map.between(tmp_pose2);
            tf::Transform diff_tf;
            diff_tf = xythetaToTF(diff_pose2.x(), diff_pose2.y(), diff_pose2.theta());

            tf::Transform first_guess_tf =
              laser_to_base_ * diff_tf * base_to_laser_;

            sm_icp_params_.first_guess[0] = first_guess_tf.getOrigin().getX();
            sm_icp_params_.first_guess[1] = first_guess_tf.getOrigin().getY();
            sm_icp_params_.first_guess[2] = tf::getYaw(first_guess_tf.getRotation());

            // Do scan matching for new factor
            sm_icp(&sm_icp_params_, &sm_icp_result_);

            // Get tf of scan matching result and transform to map frame
            tf::Transform pose_diff_tf;
            pose_diff_tf = xythetaToTF(sm_icp_result_.x[0],
                                       sm_icp_result_.x[1],
                                       sm_icp_result_.x[2]);

            // Transform to base_link frame
            pose_diff_tf = base_to_laser_ * pose_diff_tf * laser_to_base_;

            // Save loop closing tf as Pose2
            Pose2 lc_mean(pose_diff_tf.getOrigin().getX(),
                          pose_diff_tf.getOrigin().getY(),
                          tf::getYaw(pose_diff_tf.getRotation()));

            noiseModel::Diagonal::shared_ptr scan_match_noise =
              noiseModel::Diagonal::Variances(
                  (Vector(3) << gsl_matrix_get(sm_icp_result_.cov_x_m, 0, 0),
                                gsl_matrix_get(sm_icp_result_.cov_x_m, 1, 1),
                                gsl_matrix_get(sm_icp_result_.cov_x_m, 2, 2)));

            // Add new factor between current_pose2_ and node i
            graph_.add(BetweenFactor<Pose2>(node_counter_, i, lc_mean,
                       scan_match_noise));
            // We want only one loop closing (oldest) TODO: find better solution
            break;
          }
        }
      }

      // Optimize factor graph
      ROS_INFO("Optimisation started!");
      isam_.update(graph_, new_estimates_);
      isam_.update();
      pose_estimates_ = isam_.calculateEstimate();
      ROS_INFO("Optimisation finished!");

      graph_.resize(0);
      new_estimates_.clear();

      // Save current uncertainty
      uncertainty_matrices_path_.push_back(isam_.marginalCovariance(node_counter_));
      // std::cout << isam_.marginalCovariance(node_counter_) << std::endl;

      // Create a path msg of the graph node estimates
      nav_msgs::Path new_path;
      graph_path_ = new_path;
      graph_path_.header.frame_id = "/map";
      Pose2 tmp_pose2;

      // Iterate over all node estimates
      for (unsigned int i = 0; i < pose_estimates_.size(); i++){
        // Cast Pose2 from Value
        tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));

        // Create PoseStamped variables from Pose 2 and add to path
        graph_path_.poses.push_back(pose2ToPoseStamped(tmp_pose2));
      }

      // Publish the graph
      path_pub_.publish(graph_path_);

      // Update map to odom transform
      last_pose2_map_ = *dynamic_cast<const Pose2*>(&pose_estimates_.at(node_counter_));
      latest_pose_estimate_ = pose2ToPoseStamped(last_pose2_map_);
      latest_pose_estimate_.header.stamp = latest_scan_msg_.header.stamp;

      tf::Transform last_pose_tf;
      last_pose_tf = xythetaToTF(last_pose2_map_.x(), last_pose2_map_.y(), last_pose2_map_.theta());

      map_to_odom_tf_ = last_pose_tf * odom_transform;

      // Update variables
      node_counter_ += 1;
      prev_pose2_odom_ = current_pose2_odom_;

      // Calculate Map
      if (const_map_update_steps_ && (node_counter_ - 1) % 50 == 0){
        // Draw the map
        drawMap(pose_estimates_, keyframe_ldp_vec_);
      }
      else {
        updateMap(pose_estimates_, keyframe_ldp_vec_);
        map_pub_for_static_scan_comb_.publish(occupancy_grid_msg_);
      }
    }
  }

  // Update map to odom tf
  if (new_node_){
    map_br_.sendTransform(tf::StampedTransform(map_to_odom_tf_, ros::Time::now(),
                        "map", "odom"));
    new_node_ = false;
  }else{
    Pose2 between_pose2;
    Pose2 temp_last_pose2;
    between_pose2 = prev_pose2_odom_.between(current_pose2_odom_);
    temp_last_pose2 = last_pose2_map_ * between_pose2;

    tf::Transform last_pose_tf;
    last_pose_tf = xythetaToTF(temp_last_pose2.x(),
                               temp_last_pose2.y(),
                               temp_last_pose2.theta());

    map_to_odom_tf_ = last_pose_tf * odom_transform;

    map_br_.sendTransform(tf::StampedTransform(map_to_odom_tf_, ros::Time::now(),
                        "map", "odom"));
  }

  // Calculate first Map
  if (node_counter_ == 1 && !first_map_calculated_){
    // Draw the map
    drawMap(pose_estimates_, keyframe_ldp_vec_);
    first_map_calculated_ = true;
    map_pub_for_static_scan_comb_.publish(occupancy_grid_msg_);
  }
}

bool GraphOptimiser::getMapServiceCallback(
    crowdbot_active_slam::get_map::Request &request,
    crowdbot_active_slam::get_map::Response &response){
  // Return current occupancy grid map
  response.map_msg = occupancy_grid_msg_;
  return true;
}

bool GraphOptimiser::currentPoseNodeServiceCallback(
    crowdbot_active_slam::current_pose::Request &request,
    crowdbot_active_slam::current_pose::Response &response){
  response.current_pose = latest_pose_estimate_;
  return true;
}

void GraphOptimiser::pubMap(){
  ros::NodeHandle nh;
  ros::Rate map_pub_rate(0.5);
  while (ros::ok()){
    map_pub_.publish(occupancy_grid_msg_);
    map_pub_rate.sleep();
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "graph_optimisation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  GraphOptimiser optimiser(nh, nh_);

  ros::spin();

  return 0;
}
