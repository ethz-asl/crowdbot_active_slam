#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <static_laser_scan_combiner.h>


class StaticLaserScanCombinerNodelet : public nodelet::Nodelet{
  public:
    StaticLaserScanCombinerNodelet();
    ~StaticLaserScanCombinerNodelet();
    virtual void onInit();

  private:
    boost::shared_ptr<StaticLaserScanCombiner> static_laser_scan_combiner_;
};
