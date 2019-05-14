/**
 *  remove_people_from_pcl.cpp
 *
 *  Created on: 14.05.2019
 *      Author: Dario Mammolo
 */

#include <remove_people_from_pcl.h>

RemovePeopleFromPCL::RemovePeopleFromPCL(ros::NodeHandle nh, ros::NodeHandle nh_):
  nh_(nh), nh_private_(nh_)
{
  ROS_INFO("Started RemovePeopleFromPCL");

  // Get params
  nh_private_.param<std::string>("pcl_callback_topic", pcl_callback_topic_,
                                                  "/camera/depth/color/points");
  nh_private_.param<std::string>("obstacle_callback_topic",
              obstacle_callback_topic_, "/static_scan_extractor/dyn_obstacles");
  nh_private_.param<float>("max_dist_from_robot", max_dist_from_robot_, 3.5);
  sq_max_dist_from_robot_ = pow(max_dist_from_robot_, 2);

  // Publisher
  removed_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points_removed_dyn", 1);

  // Time synchronizer subscriber
  pcl_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>
                                                (nh_, pcl_callback_topic_, 10);
  obstacle_sub_ = new message_filters::Subscriber<costmap_converter::ObstacleArrayMsg>
                                            (nh_, obstacle_callback_topic_, 10);
  int q = 10; // queue size
  sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(q), *pcl_sub_, *obstacle_sub_);
  sync_->registerCallback(boost::bind(&RemovePeopleFromPCL::pclCallback, this, _1, _2));
  ros::Duration(1.0).sleep();
}

RemovePeopleFromPCL::~RemovePeopleFromPCL()
{
  delete pcl_sub_;
  delete obstacle_sub_;
  delete sync_;
}

void RemovePeopleFromPCL::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg,
                                      const costmap_converter::ObstacleArrayMsg::ConstPtr& obstacle_msg)
{
  sensor_msgs::PointCloud2 transformed_pcl2;
  const std::string map_frame = "/map";
  const std::string cam_frame = "/camera_depth_optical_frame";

  // Get current position
  tf::StampedTransform map_to_cam_tf;
  bool got_transform = false;
  while (!got_transform){
    try
    {
      got_transform = map_to_cam_listener_.waitForTransform(map_frame,
                     cam_frame, obstacle_msg->header.stamp, ros::Duration(1.0));
      map_to_cam_listener_.lookupTransform(map_frame, cam_frame,
                                    obstacle_msg->header.stamp, map_to_cam_tf);
    }
    catch (tf::TransformException ex){
      ROS_WARN("Could not get initial transform from map to camera frame, %s", ex.what());
    }
  }

  // Transform cloud to map frame
  pcl_ros::transformPointCloud(map_frame, map_to_cam_tf, *pcl_msg, transformed_pcl2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_pcl2, *transformed_pcl);

  // Check if obstacles are close enough and remove them from cloud
  for (size_t i = 0; i < obstacle_msg->obstacles.size(); i++)
  {
    float sq_dist_to_robot = pow(map_to_cam_tf.getOrigin().getX() -
                              obstacle_msg->obstacles[i].polygon.points[0].x, 2)
                           + pow(map_to_cam_tf.getOrigin().getY() -
                              obstacle_msg->obstacles[i].polygon.points[0].y, 2);

    // Check if obstacle is close enough to be considered
    if (sq_dist_to_robot < sq_max_dist_from_robot_)
    {
      // Extract indices
      pcl::PointIndices::Ptr indices(new pcl::PointIndices());
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      for (size_t j = 0; j < (*transformed_pcl).size(); j++)
      {
        if (pow(transformed_pcl->points[j].x - obstacle_msg->obstacles[i].polygon.points[0].x, 2)
          + pow(transformed_pcl->points[j].y - obstacle_msg->obstacles[i].polygon.points[0].y, 2)
          < 0.5 * 0.5)
        {
          indices->indices.push_back(j);
        }
      }
      extract.setInputCloud(transformed_pcl);
      extract.setIndices(indices);
      extract.setNegative(true);
      extract.filter(*transformed_pcl);
    }
  }

  pcl::toROSMsg(*transformed_pcl, transformed_pcl2);
  pcl_ros::transformPointCloud(cam_frame, map_to_cam_tf.inverse(), transformed_pcl2, transformed_pcl2);
  removed_pcl_pub_.publish(transformed_pcl2);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "remove_people_from_pcl");

  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");

  RemovePeopleFromPCL removePeopleFromPCL(nh, nh_);

  ros::spin();
  return 0;
}
