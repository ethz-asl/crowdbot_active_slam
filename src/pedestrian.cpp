#include <pedestrian.h>

void Pedestrian::SetName(std::string name) { this->name = name; }

void Pedestrian::SetNumber(int number) { this->ped_number = number; }

void Pedestrian::SetElapsedTime(ros::Time time) { this->elapsed_time = time; }

void Pedestrian::SetCurrentPose(const nav_msgs::Odometry::ConstPtr& msg) {
  this->current_pose.x = msg->pose.pose.position.x;
  this->current_pose.y = msg->pose.pose.position.y;
  this->current_speed.x = msg->twist.twist.linear.x;
  this->current_speed.y = msg->twist.twist.linear.y;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, this->current_orientation);
  this->current_orientation.normalize();

  this->pose_hist->push_back(this->current_pose);
  this->pose_hist->pop_front();
}

void Pedestrian::SetGoal(const geometry_msgs::Vector3::ConstPtr& msg) {
  // Update the desired only if it a new goal (In fact also pub_goal updates it,
  // given that is used also when not testing)
  cout << "New goal - x:" << msg->x << " y:" << msg->y << endl;
  if (this->desired_pose.x != msg->x or this->desired_pose.y != msg->y) {
    this->desired_pose.x = msg->x;
    this->desired_pose.y = msg->y;
  }
  this->ShowGoal();
}

void Pedestrian::PubGoal(grid_map::GridMap* map) {
  bool into_obj = true;
  grid_map::Position desired_position;
  double obstacle;
  while (into_obj) {
    desired_position.x() = utils::RandomFloat(map->getLength().x() / 2.1);
    desired_position.y() = utils::RandomFloat(map->getLength().y() / 2.1);
    obstacle =
        map->atPosition("Occupied", desired_position + map->getPosition());
    if (obstacle != 1) into_obj = false;
  }
  // if(this->controlled == false){
  this->desired_pose.x = desired_position.x() + map->getPosition().x();
  this->desired_pose.y = desired_position.y() + map->getPosition().y();
  // }
  this->rosPub_goal.publish(this->desired_pose);
}

void Pedestrian::ShowGoal() {
  // First delete the old goal the spawn a new one
  gazebo_msgs::DeleteModel del_model;
  del_model.request.model_name = "goal_" + std::to_string(*this->GetNumber());
  if (this->spawned_goal) {
    do {
      this->gazebo_delete_goal->call(del_model);
      cout << del_model.response.status_message << endl;
    } while (!del_model.response.success);
  }
  gazebo_msgs::SpawnModel model;
  model.request.initial_pose.position.x = this->GetGoal().x;
  model.request.initial_pose.position.y = this->GetGoal().y;
  model.request.initial_pose.position.z = 0;
  model.request.model_name = "goal_" + std::to_string(*this->GetNumber());
  model.request.reference_frame = "world";
  model.request.robot_namespace =
      "goal_ns_" + std::to_string(*this->GetNumber());
  // Create command to read the xacro file
  std::string cmd =
      "xacro --inorder '" + ros::package::getPath("deep_interaction_modeling") +
      "/sf_model/gazebo_sim/urdf/goal.xacro' prefix:=" + model.request.robot_namespace;
  cmd += " r:=" + std::to_string(0.2);

  model.request.model_xml = utils::ParseXacro(cmd.c_str());
  do {
    this->gazebo_spawn_goal->call(model);
  } while (!model.response.success);
  this->spawned_goal = true;
}

const std::string* Pedestrian::GetName() { return &this->name; }

const int* Pedestrian::GetNumber() { return &this->ped_number; }

geometry_msgs::Vector3 Pedestrian::GetCurrentPose() {
  return this->current_pose;
}

deque<geometry_msgs::Vector3> Pedestrian::GetPoseHistory() {
  return *this->pose_hist;
}

geometry_msgs::Vector3 Pedestrian::GetSpeed() { return this->current_speed; }

geometry_msgs::Vector3 Pedestrian::GetGoal() { return this->desired_pose; }

tf::Quaternion Pedestrian::GetOrientation() {
  return this->current_orientation;
}

double Pedestrian::GetRelativeObjectDistance() { return this->distance_of_obj; }

int Pedestrian::GetRelativeObjectAngle() { return this->angle_of_obj; }

ros::Time Pedestrian::GetElapsedTime() { return this->elapsed_time; }
