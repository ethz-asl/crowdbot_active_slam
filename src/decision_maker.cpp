#include <decision_maker.h>

DecisionMaker::DecisionMaker(ros::NodeHandle nh, ros::NodeHandle nh_)
    : nh_(nh), nh_private_(nh_) {
      // Init finished bool
      finished_ = false;

      // Init service clients
      frontier_exploration_client_ = nh_.serviceClient
            <crowdbot_active_slam::get_frontier_list>
            ("/frontier_exploration/frontier_exploration_service");
      map_recalculation_client_ = nh_.serviceClient
            <crowdbot_active_slam::map_recalculation>
            ("/map_recalculation_service");
    }

DecisionMaker::~DecisionMaker() {}

void DecisionMaker::start_SLAM(){
  while (!finished_){
    crowdbot_active_slam::get_frontier_list frontier_srv;

    if (frontier_exploration_client_.call(frontier_srv))
    {
      ROS_INFO("Frontier exploration call successfull");
    }
    else
    {
      ROS_ERROR("Failed to call service frontier_exploration");
    }

    if (frontier_srv.response.frontier_list.size() == 0){
      finished_ = true;
      return;
    }

    // Action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> goal_client("move_base", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    goal_client.waitForServer();

    ROS_INFO("Action server started, sending goal.");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = frontier_srv.response.frontier_list[0].x;
    goal.target_pose.pose.position.y = frontier_srv.response.frontier_list[0].y;
    goal.target_pose.pose.position.z = 0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1;

    goal_client.sendGoal(goal);

    bool finished_before_timeout = goal_client.waitForResult(ros::Duration(120.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = goal_client.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");

    crowdbot_active_slam::map_recalculation map_srv;
    if (map_recalculation_client_.call(map_srv))
    {
      ROS_INFO("Map recalcilation call successfull");
    }
    else
    {
      ROS_ERROR("Failed to call service map_recalculation");
    }

    std::cout << "working ;)" << std::endl;


  }
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "decision_maker");
  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  DecisionMaker decision_maker(nh, nh_);
  decision_maker.start_SLAM();

  return 0;
}
