#include <pluginlib/class_list_macros.h>
#include <static_scan_extractor_nodelet.h>

PLUGINLIB_EXPORT_CLASS(StaticScanExtractorNodelet, nodelet::Nodelet)


StaticScanExtractorNodelet::StaticScanExtractorNodelet(){}
StaticScanExtractorNodelet::~StaticScanExtractorNodelet(){}

void StaticScanExtractorNodelet::onInit(){
  static_scan_extractor_.reset(
              new StaticScanExtractor(getNodeHandle(), getPrivateNodeHandle()));
}
