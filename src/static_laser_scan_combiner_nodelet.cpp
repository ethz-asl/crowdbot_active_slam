#include <pluginlib/class_list_macros.h>
#include <static_laser_scan_combiner_nodelet.h>

PLUGINLIB_EXPORT_CLASS(StaticLaserScanCombinerNodelet, nodelet::Nodelet)


StaticLaserScanCombinerNodelet::StaticLaserScanCombinerNodelet(){}
StaticLaserScanCombinerNodelet::~StaticLaserScanCombinerNodelet(){}

void StaticLaserScanCombinerNodelet::onInit(){
  static_laser_scan_combiner_.reset(new StaticLaserScanCombiner(getNodeHandle(), getPrivateNodeHandle()));
}
