#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>


int main(int argc, char **argv)
{
  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string test_results_path = package_path + "/test_results/" + argv[1];
  std::string ground_truth_map_path = package_path + "/worlds/" + argv[2];

  std::ifstream test_map_file(test_results_path.c_str());
  std::ifstream ground_truth_map_file(ground_truth_map_path.c_str());

  std::string line;
  int i = 0;
  int j = 0;
  int k = 0;
  double map_score = 0;
  double over_ref_score = 0;

  if (ground_truth_map_file.is_open()){
    if (test_map_file.is_open()){
      while (std::getline(test_map_file, line)){
        std::stringstream ss_test;
        ss_test << line;
        double p_test;
        ss_test >> p_test;

        std::getline(ground_truth_map_file, line);
        std::stringstream ss_ref;
        ss_ref << line;
        double p_ref;
        ss_ref >> p_ref;

        // Map Scoring
        if (p_test == -1){
          p_test = 0.5;
          if (p_ref == -1){
            p_ref = 0.5;
          }
          else {
            k += 1;
            p_ref /= 100;
          }
        }
        else {
          p_test /= 100;
          i += 1;
          if (p_ref == -1){
            p_ref = 0.5;
            j += 1;
            over_ref_score += std::pow(p_ref - p_test, 2);
          }
          else {
            p_ref /= 100;
          }
        }
        map_score += std::pow(p_ref - p_test, 2);
      }
    }
    else {
      ROS_INFO("Failed to open test_map_file!");
    }
  }
  else {
    ROS_INFO("Failed to open ground_truth_map_file!");
  }

  std::cout << "map_score: " << map_score << std::endl;
  std::cout << "Number of known cells: " << i << std::endl;
  std::cout << "Number of known test cells, while unknown in ref map: " << j << std::endl;
  std::cout << "score for known test cells, while unkown in ref map: " << over_ref_score << std::endl;
  std::cout << "Number of known ref cells, while unknown in test map: " << k << std::endl;
  return 0;
}
