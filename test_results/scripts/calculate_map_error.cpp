#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>


int main(int argc, char **argv)
{
  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string test_results_path = package_path + "/test_results/" + argv[1];

  std::ifstream map_file(test_results_path.c_str());
  std::string line;
  int i = 0;
  int abs_sum_error = 0;

  if (map_file.is_open()){
    while (std::getline(map_file, line)){
      std::stringstream ss;
      ss << line;
      int probability;
      ss >> probability;

      if (probability >= 0){
        if (probability <= 50){
          abs_sum_error += probability;
        }
        else {
          abs_sum_error += 100 - probability;
        }
        i += 1;
      }
    }
  }
  else {
    ROS_INFO("Failed to open file!");
  }

  std::cout << "Sum error of probabilities: " << abs_sum_error << std::endl;
  std::cout << "Number of known cells: " << i << std::endl;
  return 0;
}
