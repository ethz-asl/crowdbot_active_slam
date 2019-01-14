#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <graph_optimisation.h>


class GraphOptimisationNodelet : public nodelet::Nodelet{
  public:
    GraphOptimisationNodelet();
    ~GraphOptimisationNodelet();
    virtual void onInit();

  private:
    boost::shared_ptr<GraphOptimiser> graph_optimiser_;
};
