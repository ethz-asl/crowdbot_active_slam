/**
 *  Adapted from:
 *  https://github.com/ethz-asl/asl_pepper/blob/devel/
 *  asl_pepper_sensor_preprocessing/src/combine_laser_scans.cpp
 *  author was: Daniel Dugas
 */

#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

#include <glog/logging.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/transforms.h>


namespace combine_laser_scans {

template <class T>
class Setable {
  public:
    Setable() : is_set_(false) {};
    ~Setable() {};
    T get() const {
      if ( !is_set_ ) {
        throw std::exception("Attempted to access value which is not set.");
      }
      return data_;
    }
    void set(const T& data) { data_ = data; }
    bool is_set() const { return is_set_; }
  private:
    T data_;
    bool is_set_;
};

// \brief A thread-safe buffer for storing a value
template <class T>
class ProtectedBuffer {
 public:
  ProtectedBuffer() {};
  ~ProtectedBuffer() {};
  // \brief empties the buffered value, and copies it to value_out.
  // Returns false if the buffer was empty, true otherwise.
  bool flushValue(T &value_out) {
    mutex_.lock();
    bool value_is_set = value_is_set_;
    if ( value_is_set ) {
      value_out = protected_value_;
    }
    value_is_set_ = false;
    mutex_.unlock();
    if ( value_is_set ) {
      return true;
    }
    return false;
  }
  // \brief empties the buffered value.
  // Returns false if the buffer was empty, true otherwise.
  bool flushValue() {
    mutex_.lock();
    bool value_is_set = value_is_set_;
    value_is_set_ = false;
    mutex_.unlock();
    return value_is_set;
  }
  void setValue(const T &value) {
    mutex_.lock();
    protected_value_ = value;
    value_is_set_ = true;
    mutex_.unlock();
    return;
  }
  // Wait until a value is set and then flush.
  // if timeout_ms is 0, waits forever.
  bool waitUntilSetAndFlush(T &value_out, const size_t timeout_ms = 0) {
    size_t kDT_ms = 1;
    size_t total_time_waited_ms = 0;
    while ( true ) {
      if ( flushValue(value_out) ) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kDT_ms));
      total_time_waited_ms += kDT_ms;
      if ( timeout_ms != 0 && total_time_waited_ms >= timeout_ms ) {
        return false;
      }
    }
  }
 private:
  std::mutex mutex_;
  T protected_value_;
  bool value_is_set_ = false;
}; // class ProtectedBuffer

class LaserScanCombiner {

  public:
    explicit LaserScanCombiner(ros::NodeHandle& n, ros::NodeHandle& n_) :
      nh_(n), nh_private_(n_) {
      // Topic names.
      const std::string kScan1Topic = "scan1";
      const std::string kScan2Topic = "scan2";
      const std::string kCombinedScanTopic = "combined_scan_sync";

      // State
      tf_is_known_ = false;
      reference_scan1_is_set_ = false;
      reference_scan2_is_set_ = false;

      // Get Cropping constants
      nh_private_.getParam("front_scan_crop_angle_min", kScanFrontCropAngleMin);
      nh_private_.getParam("front_scan_crop_angle_max", kScanFrontCropAngleMax);
      nh_private_.getParam("rear_scan_crop_angle_min", kScanRearCropAngleMin);
      nh_private_.getParam("rear_scan_crop_angle_max", kScanRearCropAngleMax);

      // Publishers and subscribers.
      scan_1_sub_ = nh_.subscribe(kScan1Topic, 1, &LaserScanCombiner::scan1Callback, this);
      scan_2_sub_ = nh_.subscribe(kScan2Topic, 1, &LaserScanCombiner::scan2Callback, this);

      new_scan_1_ = false;
      new_scan_2_ = false;

      combined_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(kCombinedScanTopic, 1);
    }
    ~LaserScanCombiner() {}

  protected:
    /// \brief returns value represents whether the callback should return
    bool setTFScan1To2() {
      constexpr bool kAssumeStaticTransform = true;
      constexpr size_t kTFTimeout_ms = 1000;
      constexpr size_t kMsToNs = 1000000;

      if ( kAssumeStaticTransform ) {
        // Find the transform between 1 and 2 if it is not known yet.
        if ( !tf_is_known_ ) {
        LOG(INFO) << "Waiting for transform between scan 1 and scan 2 frames: " <<
          reference_scan1_.header.frame_id << " " << reference_scan2_.header.frame_id;
          try {
            ros::Duration kTFWait = ros::Duration(0, 200*kMsToNs);
            tf_listener_.waitForTransform(reference_scan1_.header.frame_id, reference_scan2_.header.frame_id,
                                          reference_scan1_.header.stamp + kTFWait,
                                          ros::Duration(0, kTFTimeout_ms * kMsToNs));
            tf_listener_.lookupTransform(reference_scan1_.header.frame_id, reference_scan2_.header.frame_id,
                                         reference_scan1_.header.stamp + kTFWait, tf_scan_1_to_scan_2_);
          }
          catch ( tf::TransformException &ex ) {
            LOG(ERROR) << "Error while looking up transform between scan 1 and scan 2 : " <<
              ex.what();
            return true;
          }
          tf_is_known_ = true;
          LOG(INFO) << "Transform found.";
          return true;
        }
      } else {
        LOG(ERROR) << "Dynamic transforms are not implemented.";
        ros::shutdown();
      }
      return false;
    }

    sensor_msgs::LaserScan generateEmptyOutputScan(const ros::Time& stamp) const {
      const double kIncrementUpsampling = 1. / kResolutionUpsampling;
      // Create a new scan
      sensor_msgs::LaserScan combined_scan;
      combined_scan.header = reference_scan1_.header;
      combined_scan.header.stamp = stamp;
      combined_scan.angle_increment = reference_scan1_.angle_increment * kIncrementUpsampling;
      combined_scan.time_increment = reference_scan1_.time_increment * kIncrementUpsampling;
      combined_scan.scan_time = reference_scan1_.scan_time;
      combined_scan.range_min = reference_scan1_.range_min;
      combined_scan.range_max = reference_scan1_.range_max;
      combined_scan.angle_min = reference_scan1_.angle_min; // start with the same angle
      combined_scan.ranges.resize(2*M_PI / combined_scan.angle_increment, 0.);
      combined_scan.intensities.resize(combined_scan.ranges.size(), 0.);
      combined_scan.angle_max = reference_scan1_.angle_min +
        combined_scan.angle_increment * ( combined_scan.ranges.size() - 1 ); // sh. be full circle
      return combined_scan;
    }

    void cropRearScan(sensor_msgs::LaserScan& scan_msg,
                      sensor_msgs::LaserScan& cropped_scan){
      // Find the index of the first and last points within angle crop
      double first_scan_index_after_anglemin =
          (kScanRearCropAngleMin - scan_msg.angle_min) / scan_msg.angle_increment;
      double last_scan_index_before_anglemax =
          (kScanRearCropAngleMax - scan_msg.angle_min) / scan_msg.angle_increment;
      if ( first_scan_index_after_anglemin < 0 || last_scan_index_before_anglemax < 0 ) {
              LOG(ERROR) << "Angle index should not have negative value:"
                << first_scan_index_after_anglemin << " " << last_scan_index_before_anglemax << std::endl
                << "angle_increment: " << scan_msg.angle_increment << ", angle_min: " << scan_msg.angle_min
                << ", kScanCropAngleMin: " << kScanRearCropAngleMin
                << ", kScanCropAngleMax: " << kScanRearCropAngleMax;
      }
      size_t i_first = std::max(0., std::ceil(first_scan_index_after_anglemin));
      size_t i_last = std::min(std::floor(last_scan_index_before_anglemax), scan_msg.ranges.size() - 1.);
      size_t cropped_len = i_last - i_first + 1;
      // Angles
      double angle_first = scan_msg.angle_min + i_first * scan_msg.angle_increment;
      double angle_last = angle_first + (cropped_len - 1.) * scan_msg.angle_increment;

      // Generate the scan to fill in.
      cropped_scan.header = scan_msg.header;
      cropped_scan.angle_increment = scan_msg.angle_increment;
      cropped_scan.time_increment = scan_msg.time_increment;
      cropped_scan.scan_time = scan_msg.scan_time;
      cropped_scan.range_min = scan_msg.range_min;
      cropped_scan.range_max = scan_msg.range_max;
      cropped_scan.angle_min = angle_first;
      cropped_scan.angle_max = angle_last;
      cropped_scan.ranges.resize(cropped_len);
      cropped_scan.intensities.resize(cropped_len);

      // Fill in ranges.
      for ( size_t j = 0; j < cropped_len; j++ ) {
        size_t i = j + i_first;
        if (scan_msg.ranges.at(i) < cropped_scan.range_min){
          cropped_scan.ranges.at(j) = 0;
        }
        else {
          cropped_scan.ranges.at(j) = scan_msg.ranges.at(i);
        }
      }

      // Fill in intensities. if no intensities, spoof intensities
      if ( scan_msg.intensities.size() == 0 ) {
        for ( size_t j = 0; j < cropped_len; j++ ) {
          size_t i = j + i_first;
          cropped_scan.intensities.at(j) = 1.0;
        }
      } else {
        for ( size_t j = 0; j < cropped_len; j++ ) {
          size_t i = j + i_first;
          cropped_scan.intensities.at(j) = scan_msg.intensities.at(i);
        }
      }
    }

    void scan1Callback(const sensor_msgs::LaserScan::ConstPtr& front_msg){
      new_scan_1_ = true;
      front_msg_ = *front_msg;
      if (new_scan_1_ && new_scan_2_){
        int duration = (front_msg_.header.stamp - rear_msg_.header.stamp).toSec();
        if (duration < 0) duration = -duration;
        if (duration < 0.02){
          scanCallback(front_msg_, rear_msg_);
          new_scan_1_ = false;
          new_scan_2_ = false;
        }
        else {
          ROS_WARN("combined_laser_scans: Skipping scan as stamps difference greater than 0.02");
        }
      }
    }

    void scan2Callback(const sensor_msgs::LaserScan::ConstPtr& rear_msg){
      new_scan_2_ = true;
      rear_msg_ = *rear_msg;
      if (new_scan_1_ && new_scan_2_){
        int duration = (front_msg_.header.stamp - rear_msg_.header.stamp).toSec();
        if (duration < 0) duration = -duration;
        if (duration < 0.02){
          scanCallback(front_msg_, rear_msg_);
          new_scan_1_ = false;
          new_scan_2_ = false;
        }
        else {
          ROS_WARN("combined_laser_scans: Skipping scan as stamps difference greater than 0.02");
        }
      }
    }

    void scanCallback(sensor_msgs::LaserScan& front_msg,
                      sensor_msgs::LaserScan& rear_msg){
      VLOG(3) << "scan1callback";
      // Set the reference static transform between lasers.
      if (front_msg.header.stamp < rear_msg.header.stamp){
        if ( !reference_scan1_is_set_ ) {
          reference_scan1_ = front_msg;
          reference_scan1_is_set_ = true;
          LOG(INFO) << "First scan set as reference scan for sensor 1.";
        }
        if ( !reference_scan2_is_set_ ) {
          reference_scan2_ = rear_msg;
          reference_scan2_is_set_ = true;
          LOG(INFO) << "First scan set as reference scan for sensor 2.";
        }
      }
      else {
        if ( !reference_scan2_is_set_ ) {
          reference_scan2_ = rear_msg;
          reference_scan2_is_set_ = true;
          LOG(INFO) << "First scan set as reference scan for sensor 2.";
        }
        if ( !reference_scan1_is_set_ ) {
          reference_scan1_ = front_msg;
          reference_scan1_is_set_ = true;
          LOG(INFO) << "First scan set as reference scan for sensor 1.";
        }
      }
      if ( setTFScan1To2() ) {
        return;
      }

      // Generate the scan to fill in.
      sensor_msgs::LaserScan combined_scan = generateEmptyOutputScan(front_msg.header.stamp);

      // fill values from Scan 1 to combined scan
      // as the new scan has n times the resolution, and both start with the same angle,
      // this should be equivalent to mapping original values to every other cell in the new scan.
      // (except for the last part, where no values exist in the original scan)
      CHECK( combined_scan.angle_min == front_msg.angle_min ); // sanity check
      for ( size_t i = 0; i < front_msg.ranges.size(); i++ ) {
        // Crop angles outside of desired range (valid for pepper with laptop tray)
        float angle = front_msg.angle_min + i * front_msg.angle_increment;
        if ( angle < kScanFrontCropAngleMin || angle > kScanFrontCropAngleMax ) {
          continue;
        }
        // Fill values
        if (front_msg.ranges.at(i) < combined_scan.range_min){
          combined_scan.ranges.at(i*kResolutionUpsampling) = 0;
          combined_scan.intensities.at(i*kResolutionUpsampling) = 0;
        }
        else {
          combined_scan.ranges.at(i*kResolutionUpsampling) = front_msg.ranges.at(i);
          combined_scan.intensities.at(i*kResolutionUpsampling) = front_msg.intensities.at(i);
        }
      }

      VLOG(3) << "";

      // Set the reference static transform between lasers.
      if ( setTFScan1To2() ) {
        return;
      }

      // Crop rear scan
      sensor_msgs::LaserScan cropped_rear_scan;
      cropRearScan(rear_msg, cropped_rear_scan);

      // Project LaserScan to a point cloud
      sensor_msgs::PointCloud2 temp_cloud2;
      projector_.projectLaser(cropped_rear_scan, temp_cloud2);

      // Convert the point cloud to frame 1
      sensor_msgs::PointCloud2 scan2_in_frame1;
      pcl_ros::transformPointCloud(reference_scan1_.header.frame_id, tf_scan_1_to_scan_2_,
          temp_cloud2, scan2_in_frame1);
      // Convert the point cloud to scan values (angle, distance).
      sensor_msgs::PointCloud scan2_in_frame1_xyzi;
      sensor_msgs::convertPointCloud2ToPointCloud(scan2_in_frame1, scan2_in_frame1_xyzi);
      // Find intensity values if desired
      bool found_intensities = false;
      std::vector<float> scan2_intensities;
      try {
        VLOG(3) << "";
        for ( size_t i = 0; i < scan2_in_frame1_xyzi.channels.size(); i++ ) {
          if ( scan2_in_frame1_xyzi.channels.at(i).name == "intensity" ) {
            scan2_intensities = scan2_in_frame1_xyzi.channels.at(i).values;
            if ( scan2_intensities.size() == scan2_in_frame1_xyzi.points.size() ) {
              found_intensities = true;
              break;
            }
            LOG(ERROR) << "In scan 2 cloud, mismatch between intensity and points array sizes.";
          }
        }
      } catch (const std::exception& e) {
        LOG(ERROR) << "Could not find intensities for scan 2 cloud: " << e.what();
      }

      // Fill in values from scan 2
      // Scan 2 angles
      float min_relative_angle = kScanFrontCropAngleMin - combined_scan.angle_min;
      float max_relative_angle = kScanFrontCropAngleMax - combined_scan.angle_min;
      for ( size_t i = 0; i < scan2_in_frame1_xyzi.points.size();  i++ ) {
        geometry_msgs::Point32 p = scan2_in_frame1_xyzi.points.at(i);
        float angle = atan2(p.y, p.x);
        float range = sqrt(p.y * p.y + p.x * p.x);
        // find the index in combined_scan corresponding to that angle.
      VLOG(3) << "";
        float relative_angle = angle - combined_scan.angle_min;
        // constrain relative angle to [0, 2pi[
        if ( abs(relative_angle) >= 2*M_PI ) {
          relative_angle = fmod(relative_angle, ( 2*M_PI ));
        }
        if ( relative_angle < 0 && combined_scan.angle_increment > 0 ) {
          relative_angle += 2*M_PI;
        }
        if ( relative_angle > 0 && combined_scan.angle_increment < 0 ) {
          relative_angle -= 2*M_PI;
        }
        if (relative_angle < min_relative_angle ||
            relative_angle > max_relative_angle){
          CHECK( ( relative_angle / combined_scan.angle_increment )  >= 0 );
          size_t index = round(relative_angle / combined_scan.angle_increment);
          if ( index == combined_scan.ranges.size() ) {
            index = 0;
          }
          VLOG(2) << "angle: " << angle;
          VLOG(2) << "min angle: " << combined_scan.angle_min;
          VLOG(2) << "max angle: " << combined_scan.angle_max;
          VLOG(2) << "angle inc: " << combined_scan.angle_increment;
          VLOG(2) << "rel angle: " << relative_angle;
          VLOG(2) << "index: " << index;
          combined_scan.ranges.at(index) = range;
          if ( found_intensities ) {
            combined_scan.intensities.at(index) = scan2_intensities.at(i);
          }
        }
      }

      // publish result.
      latest_published_scan_ = combined_scan;
      combined_scan_pub_.publish(combined_scan);
    }

  private:
    // ROS
    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_private_;
    ros::Subscriber scan_1_sub_;
    ros::Subscriber scan_2_sub_;
    ros::Publisher combined_scan_pub_;
    tf::TransformListener tf_listener_;
    // State
    tf::StampedTransform tf_scan_1_to_scan_2_;
    bool tf_is_known_;
    sensor_msgs::LaserScan reference_scan1_;
    bool reference_scan1_is_set_;
    sensor_msgs::LaserScan reference_scan2_;
    bool reference_scan2_is_set_;
    laser_geometry::LaserProjection projector_;
    sensor_msgs::LaserScan latest_scan1_;
    sensor_msgs::LaserScan latest_published_scan_;
    std::mutex mutex_;
    // Constant
    const size_t kResolutionUpsampling = 3; // how much finer is the combined scan vs orginal.
    double kScanFrontCropAngleMin;
    double kScanFrontCropAngleMax;
    double kScanRearCropAngleMin;
    double kScanRearCropAngleMax;
    bool new_scan_1_;
    bool new_scan_2_;
    sensor_msgs::LaserScan front_msg_;
    sensor_msgs::LaserScan rear_msg_;

}; // class LaserScanCombiner

} // namespace combine_laser_scans

using namespace combine_laser_scans;

int main(int argc, char **argv) {

  ros::init(argc, argv, "combine_laser_scans");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  LaserScanCombiner laser_scan_combiner(n, n_);

  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception.");
    return 1;
  }

  return 0;
}
