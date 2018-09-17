/**
 *  graph_optimisation.cpp
 *
 *  Created on: 13.08.2018
 *      Author: Dario Mammolo
 */

#include <graph_optimisation.h>

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

  // Initialise subscriber and publisher
  pose_sub_ = nh_.subscribe("pose2D", 1, &GraphOptimiser::scanMatcherCallback, this);
  scan_sub_ = nh_.subscribe("base_scan", 1, &GraphOptimiser::scanCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("/graph_path", 1);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_map", 1);

  // Initialize base to laser tf
  tf::StampedTransform base_to_laser_tf_;
  base_to_laser_listener_.waitForTransform("/base_link", "/laser", ros::Time(0),
                                ros::Duration(1.0));
  base_to_laser_listener_.lookupTransform("/base_link", "/laser",
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
  map_resolution_ = 0.1;
  map_width_ = 500;
  map_height_ = 500;

  // Initialise other parameters
  dist_linear_sq_ = node_dist_linear_ * node_dist_linear_;
  first_scan_pose_ = true;
  scan_callback_initialized_ = false;
  node_counter_ = 0;
}

geometry_msgs::PoseStamped GraphOptimiser::createPoseStamped(Pose2 pose2){
  geometry_msgs::Point pose_position;
  pose_position.x = pose2.x();
  pose_position.y = pose2.y();

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pose2.theta());
  geometry_msgs::Quaternion pose_orientation;
  quaternionTFToMsg(q, pose_orientation);

  geometry_msgs::Pose pose;
  pose.position = pose_position;
  pose.orientation = pose_orientation;

  geometry_msgs::PoseStamped posestamped;
  posestamped.pose = pose;

  return posestamped;
}

void GraphOptimiser::laserScanToLDP(sensor_msgs::LaserScan& scan_msg, LDP& ldp){
  unsigned int n = scan_msg.ranges.size();
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
    ldp->theta[i] = scan_msg.angle_min + i * scan_msg.angle_increment;
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

tf::Transform xythetaToTF(double x, double y, double theta){
  tf::Transform xytheta_tf;
  xytheta_tf.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  xytheta_tf.setRotation(q);

  return xytheta_tf;
}

std::vector<int> positionToMapIndex(double x, double y,
   unsigned int width, unsigned int height, float resolution){
  std::vector<int> index(2);
  index[0] = (x - resolution / 2) / resolution + width / 2;
  index[1] = (y - resolution / 2) / resolution + height / 2;

  return index;
}

void GraphOptimiser::drawMap(gtsam::Values pose_estimates,
                             std::vector<LDP> keyframe_ldp_vec){
  // Init arrays to count hits and misses
  int hits_array[map_width_][map_height_];
  int misses_array[map_width_][map_height_];
  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      hits_array[i][j] = 0;
      misses_array[i][j] = 0;
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
    robot_pose_index = positionToMapIndex(pose2_estimate.x(),
                                          pose2_estimate.y(),
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

      std::vector<int> end_point_index;
      end_point_index = positionToMapIndex(endpoint.getOrigin().getX(),
                                           endpoint.getOrigin().getY(),
                                           map_width_,
                                           map_height_,
                                           map_resolution_);

      // Save index in shorter variable for later use
      int x1 = end_point_index[0];
      int y1 = end_point_index[1];

      // Add the endpoint to the hits_array
      hits_array[x1][y1]++;
      misses_array[x0][y0]++;

      // Bresenham's line algorithm
      // starting from "https://github.com/lama-imr/lama_utilities/blob/indigo-devel/map_ray_caster/src/map_ray_caster.cpp"
      // "https://csustan.csustan.edu/~tom/Lecture-Notes/Graphics/Bresenham-Line/Bresenham-Line.pdf"
      int dx = x1 - x0;
      int dy = y1 - y0;
      int xstep = 1;
      int ystep = 1;
      if (dx < 0) {dx = -dx; xstep = -1;};
      if (dy < 0) {dy = -dy; ystep = -1;};

      int twodx = 2 * dx;
      int twody = 2 * dy;

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
          misses_array[x][y]++;
        }
      }
      else {
        int fraction_increment = 2 * dx;
        int fraction = 2 * dx - dy;
        int x = x0;
        int y = y0  + ystep;
        for (x, y; y != y1; y += ystep){
          fraction += fraction_increment;
          if (fraction >= 0){
            x += xstep;
            fraction -= twody;
          }
          misses_array[x][y]++;
        }
      }
    }
  }

  int occupancy_probability_array[map_width_][map_height_];
  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      if (hits_array[i][j] > 0 || misses_array[i][j] > 0){
        occupancy_probability_array[i][j] = 100.0*(float(hits_array[i][j]) /
                                  float(hits_array[i][j] + misses_array[i][j]));
      }
      else{
        occupancy_probability_array[i][j] = 50;
      }
    }
  }

  nav_msgs::OccupancyGrid occupancy_grid_msg;
  occupancy_grid_msg.header.frame_id = "map";
  occupancy_grid_msg.info.resolution = map_resolution_;
  occupancy_grid_msg.info.width = map_width_;
  occupancy_grid_msg.info.height = map_height_;

  // Origin
  geometry_msgs::Point pose_position;
  pose_position.x = -(map_width_ * map_resolution_ / 2);
  pose_position.y = -(map_height_ * map_resolution_ / 2);
  pose_position.z = 0;

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
  geometry_msgs::Quaternion pose_orientation;
  quaternionTFToMsg(q, pose_orientation);

  geometry_msgs::Pose origin_pose;
  origin_pose.position = pose_position;
  origin_pose.orientation = pose_orientation;

  occupancy_grid_msg.info.origin = origin_pose;

  std::vector<int8_t> data;
  for (int i = 0; i < map_width_; i++){
    for (int j = 0; j < map_height_; j++){
      data.push_back(occupancy_probability_array[j][i]);
      //data.push_back(hits_array[i][j] / (hits_array[i][j] + misses_array[i][j]));
      //data.push_back(0);
    }
  }
  occupancy_grid_msg.data = data;
  map_pub_.publish(occupancy_grid_msg);
}

void GraphOptimiser::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  latest_scan_msg_ = *scan_msg;
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
  // TODO: Pose2 next_mean = prev_pose2_.between(current_pose2_); does the same
  double x_diff = current_pose2_.x() - prev_pose2_.x();
  double y_diff = current_pose2_.y() - prev_pose2_.y();
  double diff_dist_linear_sq = x_diff * x_diff + y_diff * y_diff;
  double angle_diff = current_pose2_.theta() - prev_pose2_.theta();

  // Add a new node if robot moved far enough
  if ((diff_dist_linear_sq > dist_linear_sq_) or
     (std::abs(angle_diff) > node_dist_angular_)){
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

    // look for loop closings factors
    double x_high = current_pose2_estimate.x() + lc_radius_;
    double x_low = current_pose2_estimate.x() - lc_radius_;
    double y_high = current_pose2_estimate.y() + lc_radius_;
    double y_low = current_pose2_estimate.y() - lc_radius_;
    double lc_radius_squared = lc_radius_ * lc_radius_;

    for (int i = 0; i < node_counter_ - 1; i++){
      Pose2 tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));
      if (tmp_pose2.x() <= x_high && tmp_pose2.x() >= x_low &&
          tmp_pose2.y() <= y_high && tmp_pose2.y() >= y_low){
        double x_diff = current_pose2_estimate.x() - tmp_pose2.x();
        double y_diff = current_pose2_estimate.y() - tmp_pose2.y();
        if (x_diff * x_diff + y_diff * y_diff <= lc_radius_squared){
          // Add new factor between current_pose2_ and node i
          sm_icp_params_.laser_ref = current_ldp_;
          sm_icp_params_.laser_sens = keyframe_ldp_vec_[i];

          Pose2 diff_pose2 = current_pose2_estimate.between(tmp_pose2);
          tf::Transform diff_tf;
          diff_tf = xythetaToTF(diff_pose2.x(), diff_pose2.y(), diff_pose2.theta());

          tf::Transform first_guess_tf =
          laser_to_base_ * diff_tf * base_to_laser_;

          sm_icp_params_.first_guess[0] = first_guess_tf.getOrigin().getX();
          sm_icp_params_.first_guess[1] = first_guess_tf.getOrigin().getY();
          sm_icp_params_.first_guess[2] = tf::getYaw(first_guess_tf.getRotation());

          sm_icp(&sm_icp_params_, &sm_icp_result_);

          tf::Transform pose_diff_tf;
          pose_diff_tf = xythetaToTF(sm_icp_result_.x[0],
                                     sm_icp_result_.x[1],
                                     sm_icp_result_.x[2]);

          pose_diff_tf = base_to_laser_ * pose_diff_tf * laser_to_base_;

          Pose2 lc_mean(pose_diff_tf.getOrigin().getX(),
                        pose_diff_tf.getOrigin().getY(),
                        tf::getYaw(pose_diff_tf.getRotation()));
          lc_mean.print();
          graph_.add(BetweenFactor<Pose2>(node_counter_, i, lc_mean,
                     scan_match_noise_));
          std::cout << "Found loop closing and added to graph" << std::endl;
        }
      }
    }

    // Optimize the graph
    LevenbergMarquardtOptimizer optimizer(graph_, pose_estimates_);
    ROS_INFO("Optimisation started!");
    pose_estimates_ = optimizer.optimize();
    ROS_INFO("Optimisation finished!");
    pose_estimates_.print("Pose estimates:\n");

    // Create a path msg of the graph node estimates
    nav_msgs::Path new_path;
    graph_path_ = new_path;
    graph_path_.header.frame_id = "/map";
    Pose2 tmp_pose2;
    geometry_msgs::PoseStamped tmp_posestamped;

    // Iterate over all node estimates
    for (int i = 0; i < pose_estimates_.size(); i++){
      // Cast Pose2 from Value
      tmp_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(i));

      // Create PoseStamped variables from Pose 2 and add to path
      tmp_posestamped = createPoseStamped(tmp_pose2);
      graph_path_.poses.push_back(tmp_posestamped);
    }

    // Publish the graph
    path_pub_.publish(graph_path_);

    // Update map to odom transform
    Pose2 last_pose2;
    last_pose2 = *dynamic_cast<const Pose2*>(&pose_estimates_.at(node_counter_));

    tf::Transform last_pose_tf;
    last_pose_tf = xythetaToTF(last_pose2.x(), last_pose2.y(), last_pose2.theta());

    map_to_odom_tf_ = last_pose_tf * odom_transform;

    // Update variables
    node_counter_ += 1;
    prev_pose2_ = current_pose2_;
  }
  // Update map to odom tf
  map_br_.sendTransform(tf::StampedTransform(map_to_odom_tf_, ros::Time::now(),
                        "map", "odom"));

  // Draw the map
  drawMap(pose_estimates_, keyframe_ldp_vec_);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "GraphOptimiser");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  GraphOptimiser optimiser(nh, nh_);
  ros::spin();
  return 0;
}
