/**
*  helper.h
*
*  Created on: 17.09.2018
*      Author: Dario Mammolo
*/

#include <graph_optimisation.h>

using namespace gtsam;


/**
 *  A helper function which creates PoseStamped msg from Pose2.
 */
geometry_msgs::PoseStamped pose2ToPoseStamped(Pose2 pose2){
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
 *  A helper function which creates map index from position information.
 */
std::vector<int> positionToMapIndex(double x, double y,
   unsigned int width, unsigned int height, float resolution){
  std::vector<int> index(2);
  index[0] = (x - resolution / 2) / resolution + width / 2;
  index[1] = (y - resolution / 2) / resolution + height / 2;

  return index;
}

/**
 *  A helper function which creates Pose msg from x, y, theta information.
 */
geometry_msgs::Pose xythetaToPose(double x, double y, double theta){
  geometry_msgs::Point pose_position;
  pose_position.x = x;
  pose_position.y = y;
  pose_position.z = 0;

  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, 0);
  geometry_msgs::Quaternion pose_orientation;
  quaternionTFToMsg(q, pose_orientation);

  geometry_msgs::Pose pose;
  pose.position = pose_position;
  pose.orientation = pose_orientation;

  return pose;
}
