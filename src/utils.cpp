#include <utils.h>

/*This function is used to parse the xacro file and transform it in urdf.*/
// ------------------------------------------------------------------------
std::string utils::ParseXacro(const char* cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  if (!pipe) throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get())) {
    if (fgets(buffer.data(), 128, pipe.get()) != NULL) result += buffer.data();
  }
  return result;
}
// ------------------------------------------------------------------------

/*This function calculates random points used to spawn the models*/
// ------------------------------------------------------------------------
void utils::PedestrianStartingPose(double limit_x, double limit_y,
                                   std::vector<double>* x,
                                   std::vector<double>* y,
                                   grid_map::GridMap* map) {
  int number = x->size();
  int index = 0;
  double obstacle = 0;
  grid_map::Position ped_position;
  while (index < number) {
    ped_position.x() = utils::RandomFloat(limit_x);
    ped_position.y() = utils::RandomFloat(limit_y);
    obstacle = map->atPosition("Occupied", ped_position + map->getPosition());
    if (obstacle != 1 && !(CheckIfObstacle(ped_position, map)) &&
        !(CheckOccupied(x, y, ped_position.x(), ped_position.y()))) {
      x->at(index) = ped_position.x() + map->getPosition().x();
      y->at(index) = ped_position.y() + map->getPosition().y();
      index++;
    }
  }
}

bool utils::CheckIfObstacle(grid_map::Position position,
                            grid_map::GridMap* map) {
  double closest_x, closest_y;
  closest_x = map->atPosition("Closest X", position + map->getPosition());
  closest_y = map->atPosition("Closest Y", position + map->getPosition());
  if (abs(closest_y) <= 0.5 || abs(closest_x) <= 0.5) {
    return true;
  } else {
    return false;
  }
}

// Check if there is already a pedestrian in the same place
bool utils::CheckOccupied(std::vector<double>* x, std::vector<double>* y,
                          double x_c, double y_c) {
  for (int i = 0; i < x->size(); i++) {
    if (pow((x->at(i) - x_c), 2) + pow((y->at(i) - y_c), 2) <=
        pow(2 * 0.2, 2)) {
      return true;
    }
  }
  return false;
}
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
double utils::RandomFloat(double N) {
  double r = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
  return static_cast<double>(-N) +
         r * (static_cast<double>(N) - static_cast<double>(-N));
}
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
double utils::CalcMod(double x, double y) {
  return sqrt(pow(x, 2) + pow(y, 2));
}
// ------------------------------------------------------------------------

/*This function calculates the direction of motion of the pedestrian*/
// ------------------------------------------------------------------------
geometry_msgs::Vector3 utils::CalcDirection(geometry_msgs::Vector3 current,
                                            geometry_msgs::Vector3 desired) {
  geometry_msgs::Vector3 e_t;
  double mod_e_t = utils::CalcMod(desired.x - current.x, desired.y - current.y);
  if (mod_e_t != 0) {
    e_t.x = (desired.x - current.x) / mod_e_t;
    e_t.y = (desired.y - current.y) / mod_e_t;
  } else {
    e_t.x = 0;
    e_t.y = 0;
  }
  e_t.z = 0;

  return e_t;
}
// ------------------------------------------------------------------------

/*This function applies a rotation matrix*/
// ------------------------------------------------------------------------
geometry_msgs::Vector3 utils::RotateOfAngle(double x, double y, double theta) {
  geometry_msgs::Vector3 rotated;
  rotated.x = x * cos(theta) - y * sin(theta);
  rotated.y = x * sin(theta) + y * cos(theta);
  return rotated;
}
// ------------------------------------------------------------------------
