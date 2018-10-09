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

  // Initialise map to odom tf
  tf::Transform init_tf;
  init_tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion init_q;
  init_q.setRPY(0.0, 0.0, 0);
  init_tf.setRotation(init_q);
  map_to_odom_tf_ = init_tf;
  map_br_.sendTransform(tf::StampedTransform(map_to_odom_tf_, ros::Time::now(),
                        "map", "odom"));
}

GraphOptimiser::~GraphOptimiser(){}

void GraphOptimiser::initParams(){
  // Get params which describe in which distance the node of the graph is built
  nh_private_.param<double>("node_dist_linear", node_dist_linear_, 0.10);
  nh_private_.param<double>("node_dist_angular", node_dist_angular_, 0.175);
  nh_private_.param<double>("loop_closing_radius", lc_radius_, 0.175);
  nh_private_.param<bool>("const_map_update_steps", const_map_update_steps_, false);

  // Initialise subscriber and publisher
  pose_sub_ = nh_.subscribe("pose2D", 1, &GraphOptimiser::scanMatcherCallback, this);
  scan_sub_ = nh_.subscribe("base_scan", 1, &GraphOptimiser::scanCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("graph_path", 1);
  action_path_pub_ = nh_.advertise<nav_msgs::Path>("action_graph", 1);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1);

  // Initialise map service
  map_service_ = nh_.advertiseService("map_recalculation_service",
                        &GraphOptimiser::mapRecalculationServiceCallback, this);
  utility_calc_service_ = nh_.advertiseService("utility_calc_service",
                             &GraphOptimiser::utilityCalcServiceCallback, this);

  // Initialize base to laser tf
  tf::StampedTransform base_to_laser_tf_;
  base_to_laser_listener_.waitForTransform("base_link", "laser", ros::Time(0),
                                ros::Duration(1.0));
  base_to_laser_listener_.lookupTransform("base_link", "laser",
                                  ros::Time(0), base_to_laser_tf_);
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
  sm_icp_params_.max_correspondence_dist = 0.3;
  sm_icp_params_.use_corr_tricks = 1;
  sm_icp_params_.restart = 0;
  sm_icp_params_.restart_threshold_mean_error = 0.01;
  sm_icp_params_.restart_dt = 1.0;
  sm_icp_params_.restart_dtheta = 0.1;
  sm_icp_params_.outliers_maxPerc = 0.9;
  sm_icp_params_.outliers_adaptive_order = 0.7;
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
  sm_icp_params_.do_compute_covariance = 0;
  sm_icp_params_.debug_verify_tricks = 0;
  sm_icp_params_.sigma = 0.01;

  // Initialise noise on scan matching nodes
  scan_match_noise_ = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, 0.2));

  // Map paramters
  map_resolution_ = 0.05;
  map_width_ = 1000;
  map_height_ = 1000;
  log_odds_array_ = Eigen::MatrixXf(map_width_, map_height_);
  l_0_ = log(0.5 / 0.5);
  p_occ_ = 0.997;
  p_free_ = 0.003;
  l_occ_ = log(p_occ_ / (1.0 - p_occ_));
  l_free_ = log(p_free_ / (1.0 - p_free_));

  // Initialise other parameters
  dist_linear_sq_ = node_dist_linear_ * node_dist_linear_;
  first_scan_pose_ = true;
  scan_callback_initialized_ = false;
  new_node_ = false;
  node_counter_ = 0;
}

void GraphOptimiser::laserScanToLDP(sensor_msgs::LaserScan& scan_msg, LDP& ldp){
  unsigned int n = scan_ranges_size_;
  ldp = ld_alloc_new(n);

  for (int i = 0; i < n; i++){
    double r = scan_msg.ranges[i];

    if (r > scan_msg.range_min && r < scan_msg.range_max){
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
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

void GraphOptimiser::getSubsetOfMap(nav_msgs::Path action_path,
                                    std::vector<double> alpha,
                                    std::map<int, double>& subset){
  //
  int node_size = action_path.poses.size();
  for (int i = 0; i < node_size; i++){
    // Get pose of laser
    tf::Pose robot_tf;
    tf::poseMsgToTF(action_path.poses[i].pose, robot_tf);
    tf::Transform laser_tf;
    laser_tf = robot_tf * base_to_laser_;
    double robot_x = laser_tf.getOrigin().getX();
    double robot_y = laser_tf.getOrigin().getY();

    std::vector<int> laser_idx = positionToMapIndex(robot_x, robot_y,
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
          // Check if x,y still in map
          if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_){
            int temp_id = mapIndexToId(x, y, map_width_);
            // Check if next cell is a wall
            if (int(occupancy_grid_msg_.data[temp_id]) < 90){
              // Check if already in subset
              if (subset.find(temp_id) != subset.end()){
                subset.find(temp_id)->second = alpha[i];
              }
              else {
                subset.insert(std::make_pair(temp_id, alpha[i]));
              }
            }
            else {
              wall_reached = true;
            }
          }
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
          // Check if x,y still in map
          if (x >= 0 && x < map_width_ && y >= 0 && y < map_height_){
            int temp_id = mapIndexToId(x, y, map_width_);
            // Check if next cell is a wall
            if (int(occupancy_grid_msg_.data[temp_id]) < 90){
              // Check if already in subset
              if (subset.find(temp_id) != subset.end()){
                subset.find(temp_id)->second = alpha[i];
              }
              else {
                subset.insert(std::make_pair(temp_id, alpha[i]));
              }
            }
            else {
              wall_reached = true;
            }
          }
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
                                           std::vector<int> end_point_index_m){
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
    for (x, y; x != x1; x += xstep){
      fraction += fraction_increment;
      if (fraction >= 0){
        y += ystep;
        fraction -= twodx;
      }
      if (x != end_point_index_m[0] || y != end_point_index_m[1]){
        log_odds_array_(x, y) += l_free_ - l_0_;
      }
    }
  }
  else {
    int fraction_increment = 2 * dx;
    int fraction = 2 * dx - dy;
    int x = x0;
    int y = y0 + ystep;
    for (x, y; y != y1; y += ystep){
      fraction += fraction_increment;
      if (fraction >= 0){
        x += xstep;
        fraction -= twody;
      }
      if (x != end_point_index_m[0] || y != end_point_index_m[1]){
        log_odds_array_(x, y) += l_free_ - l_0_;
      }
    }
  }
}

void GraphOptimiser::drawMap(gtsam::Values pose_estimates,
                             std::vector<LDP> keyframe_ldp_vec){
  // Init arrays to l_0_
  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      log_odds_array_(i, j) = l_0_;
    }
  }

  // Loop over each keyframe
  for (int i = 0; i < keyframe_ldp_vec.size(); i++){
    Pose2 pose2_estimate = *dynamic_cast<const Pose2*>(&pose_estimates.at(i));
    tf::Transform map_to_laser_tf;
    map_to_laser_tf = xythetaToTF(pose2_estimate.x(),
                                  pose2_estimate.y(),
                                  pose2_estimate.theta()) * base_to_laser_;

    std::vector<int> robot_pose_index;

    double robot_x = pose2_estimate.x();
    double robot_y = pose2_estimate.y();
    robot_pose_index = positionToMapIndex(robot_x,
                                          robot_y,
                                          map_width_,
                                          map_height_,
                                          map_resolution_);

    // Save index in shorter variable for later use
    int x0 = robot_pose_index[0];
    int y0 = robot_pose_index[1];

    // Loop over each ray of the scan
    for (int j = 0; j < keyframe_ldp_vec[i]->nrays; j++){
      double reading = keyframe_ldp_vec[i]->readings[j];
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
      log_odds_array_(x1, y1) += l_occ_ - l_0_;

      // Check if +/- 0.5*resolution is in a new cell
      double xdiff = endpoint_x - robot_x;
      double ydiff = endpoint_y - robot_y;
      double robot_laser_dist = sqrt(xdiff * xdiff + ydiff * ydiff);

      // Calculate delta_x/y with similar triangles
      double delta_x = 0.5 * map_resolution_ / robot_laser_dist * xdiff;
      double delta_y = 0.5 * map_resolution_ / robot_laser_dist * ydiff;

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
        log_odds_array_(end_point_index_p[0], end_point_index_p[1]) += l_occ_ - l_0_;
      }
      if (end_point_index_m[0] != x1 || end_point_index_m[1] != y1){
        log_odds_array_(end_point_index_m[0], end_point_index_m[1]) += l_occ_ - l_0_;
      }

      updateLogOdsWithBresenham(x0, y0, x1, y1, end_point_index_m);
    }
  }

  // Init OccupancyGrid msg
  nav_msgs::OccupancyGrid occupancy_grid_msg;
  occupancy_grid_msg.header.frame_id = "map";
  occupancy_grid_msg.info.resolution = map_resolution_;
  occupancy_grid_msg.info.width = map_width_;
  occupancy_grid_msg.info.height = map_height_;

  // Origin
  geometry_msgs::Pose origin_pose;
  origin_pose = xythetaToPose(-(map_width_ * map_resolution_ / 2),
                              -(map_height_ * map_resolution_ / 2),
                              0);
  occupancy_grid_msg.info.origin = origin_pose;

  // Transform occupancy_probability_array to ROS msg
  std::vector<int8_t> data;
  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      if (log_odds_array_(j, i) == l_0_){
        data.push_back(-1);
      }else {
        data.push_back(100 * (1.0 - 1.0 / (1.0 + exp(log_odds_array_(j, i)))));
      }
    }
  }
  occupancy_grid_msg.data = data;
  occupancy_grid_msg_ = occupancy_grid_msg;
}

void GraphOptimiser::updateMap(gtsam::Values pose_estimates,
                               std::vector<LDP> keyframe_ldp_vec){
  int i = keyframe_ldp_vec.size() - 1;
  Pose2 pose2_estimate = *dynamic_cast<const Pose2*>(&pose_estimates.at(i));
  tf::Transform map_to_laser_tf;
  map_to_laser_tf = xythetaToTF(pose2_estimate.x(),
                               pose2_estimate.y(),
                               pose2_estimate.theta()) * base_to_laser_;

  std::vector<int> robot_pose_index;

  double robot_x = pose2_estimate.x();
  double robot_y = pose2_estimate.y();
  robot_pose_index = positionToMapIndex(robot_x,
                                        robot_y,
                                        map_width_,
                                        map_height_,
                                        map_resolution_);

  // Save index in shorter variable for later use
  int x0 = robot_pose_index[0];
  int y0 = robot_pose_index[1];

  // Loop over each ray of the scan
  for (int j = 0; j < keyframe_ldp_vec[i]->nrays; j++){
    double reading = keyframe_ldp_vec[i]->readings[j];
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
    log_odds_array_(x1, y1) += l_occ_ - l_0_;

    // Check if +/- 0.5*resolution is in a new cell
    double xdiff = endpoint_x - robot_x;
    double ydiff = endpoint_y - robot_y;
    double robot_laser_dist = sqrt(xdiff * xdiff + ydiff * ydiff);

    // Calculate delta_x/y with similar triangles
    double delta_x = 0.5 * map_resolution_ / robot_laser_dist * xdiff;
    double delta_y = 0.5 * map_resolution_ / robot_laser_dist * ydiff;

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
      log_odds_array_(end_point_index_p[0], end_point_index_p[1]) += l_occ_ - l_0_;
    }
    if (end_point_index_m[0] != x1 || end_point_index_m[1] != y1){
      log_odds_array_(end_point_index_m[0], end_point_index_m[1]) += l_occ_ - l_0_;
    }

    updateLogOdsWithBresenham(x0, y0, x1, y1, end_point_index_m);
  }

  // Init OccupancyGrid msg
  nav_msgs::OccupancyGrid occupancy_grid_msg;
  occupancy_grid_msg.header.frame_id = "map";
  occupancy_grid_msg.info.resolution = map_resolution_;
  occupancy_grid_msg.info.width = map_width_;
  occupancy_grid_msg.info.height = map_height_;

  // Origin
  geometry_msgs::Pose origin_pose;
  origin_pose = xythetaToPose(-(map_width_ * map_resolution_ / 2),
                           -(map_height_ * map_resolution_ / 2),
                           0);
  occupancy_grid_msg.info.origin = origin_pose;

  // Transform occupancy_probability_array to ROS msg
  std::vector<int8_t> data;
  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      if (log_odds_array_(j, i) == l_0_){
        data.push_back(-1);
      }else {
        data.push_back(100 * (1.0 - 1.0 / (1.0 + exp(log_odds_array_(j, i)))));
      }
    }
  }
  occupancy_grid_msg.data = data;
  occupancy_grid_msg_ = occupancy_grid_msg;
}

bool GraphOptimiser::mapRecalculationServiceCallback(
  crowdbot_active_slam::map_recalculation::Request &request,
  crowdbot_active_slam::map_recalculation::Response &response){
  // Optimize the graph
  LevenbergMarquardtOptimizer optimizer(graph_, pose_estimates_);
  ROS_INFO("Optimisation started!");
  pose_estimates_ = optimizer.optimize();
  ROS_INFO("Optimisation finished!");

  // Draw the whole map
  drawMap(pose_estimates_, keyframe_ldp_vec_);

  return true;
  }

bool GraphOptimiser::utilityCalcServiceCallback(
  crowdbot_active_slam::utility_calc::Request &request,
  crowdbot_active_slam::utility_calc::Response &response){

  Pose2 current_estimate;
  int size = pose_estimates_.size();

  // Cast Pose2 from Value
  current_estimate = *dynamic_cast<const Pose2*>(&pose_estimates_.at(size - 1));

  // Get covariance from current position
  Marginals marginals(graph_, pose_estimates_);
  noiseModel::Gaussian::shared_ptr current_marginal_noise =
      noiseModel::Gaussian::Covariance(marginals.marginalCovariance(size - 1));

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

  for (int i = 0; i < request.plan.poses.size(); i++){
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

      action_graph.add(BetweenFactor<Pose2>(node_counter - 1, node_counter,
        next_mean, scan_match_noise_));
      action_estimates.insert(node_counter, next_pose);

      // Check for loop closings
      double x_high = next_pose.x() + lc_radius_;
      double x_low = next_pose.x() - lc_radius_;
      double y_high = next_pose.y() + lc_radius_;
      double y_low = next_pose.y() - lc_radius_;
      double lc_radius_squared = lc_radius_ * lc_radius_;

      for (int j = 0; j < pose_estimates_.size(); j++){
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

            noiseModel::Gaussian::shared_ptr current_marginal_noise =
                noiseModel::Gaussian::Covariance(marginals.marginalCovariance(j));

            if (map_of_lc.find(j) != map_of_lc.end()){
              int lc_index = map_of_lc.find(j)->second;
              action_graph.add(BetweenFactor<Pose2>(node_counter, Symbol(lc, lc_index),
                                              diff_pose2, scan_match_noise_));
            }
            else {
              action_graph.add(PriorFactor<Pose2>(Symbol(lc, lc_counter),
                                              tmp_pose2, current_marginal_noise));
              action_graph.add(BetweenFactor<Pose2>(node_counter, Symbol(lc, lc_counter),
                                              diff_pose2, scan_match_noise_));
              action_estimates.insert(Symbol(lc, lc_counter), tmp_pose2);

              map_of_lc.insert(std::make_pair(j, lc_counter));
              lc_counter += 1;
            }
          }
        }
      }
      node_counter += 1;
      prev_angle = angle;
      prev_pose = next_pose;
      prev_i = i;
    }
  }

  // Optimize the graph
  LevenbergMarquardtOptimizer optimizer(action_graph, action_estimates);
  ROS_INFO("Optimisation started!");
  action_estimates = optimizer.optimize();
  ROS_INFO("Optimisation finished!");

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
  Marginals action_marginals(action_graph, action_estimates);
  for (int i = 0; i < node_size; i++){
    // D-optimality
    Eigen::VectorXcd eivals = action_marginals.marginalCovariance(i).eigenvalues();
    double sum_of_logs = log(eivals[0].real()) +
                         log(eivals[1].real()) +
                         log(eivals[2].real());
    sigma_temp = exp(1.0 / 3.0 * sum_of_logs);
    alpha.push_back(1.0 + 1.0 / sigma_temp);
  }

  std::map<int, double> subset;
  getSubsetOfMap(action_path, alpha, subset);

  // Calculate utility
  double utility = 0;
  for (std::map<int, double>::iterator it = subset.begin(); it != subset.end(); ++it){
    // Get probability
    int p_percent = int(occupancy_grid_msg_.data[it->first]);
    // Check if cell is unkonwn
    if (p_percent == -1) p_percent = 50;
    // Check if not 0 or 1 to avoid nan's
    if (p_percent != 0 && p_percent != 100){
      double p = double(p_percent) / 100.0;
      utility += -(p * log2(p) + (1 - p) * log2(1 - p)) -
      1.0 / (1.0 - it->second) * log2(pow(p, it->second) + pow(1 - p, it->second));
    }
  }

  response.utility = utility;
  return true;
}

void GraphOptimiser::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  latest_scan_msg_ = *scan_msg;
  scan_ranges_size_ = latest_scan_msg_.ranges.size();
  scan_angle_increment_ = latest_scan_msg_.angle_increment;
  scan_callback_initialized_ = true;
}

void GraphOptimiser::scanMatcherCallback(const geometry_msgs::Pose2D::ConstPtr& pose2D_msg){
  // Check if scan callback initialized
  if (!scan_callback_initialized_) return;

  // Get newest transform from odom to base_link for later use
  tf::StampedTransform odom_stamped_transform;
  odom_listener_.lookupTransform("/base_link", "/odom", ros::Time(0),
                                 odom_stamped_transform);
  tf::Transform odom_transform;
  odom_transform = static_cast<tf::Transform>(odom_stamped_transform);

  // Save msg as Pose2
  current_pose2_ = Pose2((*pose2D_msg).x, (*pose2D_msg).y, (*pose2D_msg).theta);

  // Check if it is the first scan
  if (first_scan_pose_){
    // Create graph with first node
    pose_estimates_.insert(node_counter_, current_pose2_);
    noiseModel::Diagonal::shared_ptr prior_noise =
      noiseModel::Diagonal::Sigmas((Vector(3) << 0.01, 0.01, 0.01));
    graph_.add(PriorFactor<Pose2>(node_counter_, current_pose2_, prior_noise));

    // Save scan as LDP
    laserScanToLDP(latest_scan_msg_, current_ldp_);
    keyframe_ldp_vec_.push_back(current_ldp_);

    // Init some sm icp params from laser scan msg
    sm_icp_params_.min_reading = latest_scan_msg_.range_min;
    sm_icp_params_.max_reading = latest_scan_msg_.range_max;

    // Update variables
    first_scan_pose_ = false;
    prev_pose2_ = current_pose2_;
    node_counter_ += 1;
    return;
  }

  // Calculate some variables for decision if to add a new node
  double x_diff = current_pose2_.x() - prev_pose2_.x();
  double y_diff = current_pose2_.y() - prev_pose2_.y();
  double diff_dist_linear_sq = x_diff * x_diff + y_diff * y_diff;
  double angle_diff = current_pose2_.theta() - prev_pose2_.theta();

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
    Pose2 next_mean = prev_pose2_.between(current_pose2_);
    Pose2 prev_pose2_estimate = *dynamic_cast<const Pose2*>(&pose_estimates_.at(node_counter_ - 1));
    Pose2 current_pose2_estimate = prev_pose2_estimate * next_mean;
    pose_estimates_.insert(node_counter_, current_pose2_estimate);
    graph_.add(BetweenFactor<Pose2>(node_counter_ - 1, node_counter_,
      next_mean, scan_match_noise_));

    // look for loop closing factors
    double x_high = current_pose2_estimate.x() + lc_radius_;
    double x_low = current_pose2_estimate.x() - lc_radius_;
    double y_high = current_pose2_estimate.y() + lc_radius_;
    double y_low = current_pose2_estimate.y() - lc_radius_;
    double lc_radius_squared = lc_radius_ * lc_radius_;

    for (int i = 0; i < node_counter_ - 1; i++){
      Pose2 tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));

      // Check if tmp_pose2 is in square region around current pose estimate
      if (tmp_pose2.x() <= x_high && tmp_pose2.x() >= x_low &&
          tmp_pose2.y() <= y_high && tmp_pose2.y() >= y_low){
        double x_diff = current_pose2_estimate.x() - tmp_pose2.x();
        double y_diff = current_pose2_estimate.y() - tmp_pose2.y();

        // Check if tmp_pose2 is in circle around current pose estimate
        if (x_diff * x_diff + y_diff * y_diff <= lc_radius_squared){
          // Save laser scans for mapping
          sm_icp_params_.laser_ref = current_ldp_;
          sm_icp_params_.laser_sens = keyframe_ldp_vec_[i];

          // Get Pose tf between current and new Pose as first guess for icp
          Pose2 diff_pose2 = current_pose2_estimate.between(tmp_pose2);
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

          // Transform to map frame
          pose_diff_tf = base_to_laser_ * pose_diff_tf * laser_to_base_;

          // Save loop closing tf as Pose2
          Pose2 lc_mean(pose_diff_tf.getOrigin().getX(),
                        pose_diff_tf.getOrigin().getY(),
                        tf::getYaw(pose_diff_tf.getRotation()));

          // Add new factor between current_pose2_ and node i
          graph_.add(BetweenFactor<Pose2>(node_counter_, i, lc_mean,
                     scan_match_noise_));
        }
      }
    }

    if (node_counter_ % 5 == 0){
      // Optimize the graph
      LevenbergMarquardtOptimizer optimizer(graph_, pose_estimates_);
      ROS_INFO("Optimisation started!");
      pose_estimates_ = optimizer.optimize();
      ROS_INFO("Optimisation finished!");
      // pose_estimates_.print("Pose estimates:\n");
    }

    // Create a path msg of the graph node estimates
    nav_msgs::Path new_path;
    graph_path_ = new_path;
    graph_path_.header.frame_id = "/map";
    Pose2 tmp_pose2;

    // Iterate over all node estimates
    for (int i = 0; i < pose_estimates_.size(); i++){
      // Cast Pose2 from Value
      tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));

      // Create PoseStamped variables from Pose 2 and add to path
      graph_path_.poses.push_back(pose2ToPoseStamped(tmp_pose2));
    }

    // Publish the graph
    path_pub_.publish(graph_path_);

    // Update map to odom transform
    last_pose2_ = *dynamic_cast<const Pose2*>(&pose_estimates_.at(node_counter_));

    tf::Transform last_pose_tf;
    last_pose_tf = xythetaToTF(last_pose2_.x(), last_pose2_.y(), last_pose2_.theta());

    map_to_odom_tf_ = last_pose_tf * odom_transform;

    // Update variables
    node_counter_ += 1;
    prev_pose2_ = current_pose2_;

    // Calculate Map
    if (const_map_update_steps_ && (node_counter_ - 1) % 50 == 0){
      // Draw the map
      drawMap(pose_estimates_, keyframe_ldp_vec_);
    }
    else {
      updateMap(pose_estimates_, keyframe_ldp_vec_);
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
    between_pose2 = prev_pose2_.between(current_pose2_);
    temp_last_pose2 = last_pose2_ * between_pose2;

    tf::Transform last_pose_tf;
    last_pose_tf = xythetaToTF(temp_last_pose2.x(),
                               temp_last_pose2.y(),
                               temp_last_pose2.theta());

    map_to_odom_tf_ = last_pose_tf * odom_transform;

    map_br_.sendTransform(tf::StampedTransform(map_to_odom_tf_, ros::Time::now(),
                        "map", "odom"));
  }

  // Calculate first Map
  if (node_counter_ == 1){
    // Draw the map
    drawMap(pose_estimates_, keyframe_ldp_vec_);
  }

  // Publish map
  map_pub_.publish(occupancy_grid_msg_);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "graph_optimisation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  GraphOptimiser optimiser(nh, nh_);
  ros::spin();
  return 0;
}
