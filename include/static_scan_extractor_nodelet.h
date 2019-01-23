#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <static_scan_extractor.h>


class StaticScanExtractorNodelet : public nodelet::Nodelet{
  public:
    StaticScanExtractorNodelet();
    ~StaticScanExtractorNodelet();
    virtual void onInit();

  private:
    boost::shared_ptr<StaticScanExtractor> static_scan_extractor_;
};
