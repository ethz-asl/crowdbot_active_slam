#include <pluginlib/class_list_macros.h>
#include <graph_optimisation_nodelet.h>

PLUGINLIB_EXPORT_CLASS(GraphOptimisationNodelet, nodelet::Nodelet)


GraphOptimisationNodelet::GraphOptimisationNodelet(){}
GraphOptimisationNodelet::~GraphOptimisationNodelet(){}

void GraphOptimisationNodelet::onInit(){
  graph_optimiser_.reset(new GraphOptimiser(getNodeHandle(), getPrivateNodeHandle()));
}
