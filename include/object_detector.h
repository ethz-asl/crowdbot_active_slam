/**
 *  object_detector.h
 *
 *  Created on: 06.12.2018
 *      Author: Dario Mammolo
 */

#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class ObjectDetector{
public:
  /**
   *  Class Constructor
   */
  ObjectDetector();

  /**
   *  Class Constructor
   */
  ObjectDetector(double lambda, double sigma);

  /**
   *  Class Destructor
   */
  ~ObjectDetector();

  /**
   *  Detect objects in laser scans via adaptive breakpoint detector
   */
  void detectObjectsFromScan(const sensor_msgs::LaserScan& dynamic_laser_msg,
                             const sensor_msgs::LaserScan& raw_laser_msg,
                             list<map<int, double>>& normal_clusters,
                             list<map<int, double>>& occluded_clusters,
                             int upsampling_factor);

private:
  // Laser scan variables
  double lambda_;
  double sigma_;
};

#endif  // OBJECT_DETECTOR_H
