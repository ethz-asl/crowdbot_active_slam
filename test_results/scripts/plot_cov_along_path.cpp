#include <iostream>
#include <fstream>
#include <plotty/matplotlibcpp.hpp>

#include <ros/ros.h>
#include <ros/package.h>


int main(int argc, char **argv)
{
  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string test_results_path = package_path + "/test_results/" + argv[1];

  std::ifstream cov_file(test_results_path.c_str());
  std::string line;
  int i = 0;
  double xx, xy, xtheta, yy, ytheta, thetatheta;
  std::vector<double> uncertainty_vec;

  if (cov_file.is_open()){
    while (std::getline(cov_file, line)){
      if (i == 0){
        // Get new values
        std::stringstream ss;
        ss << line;
        ss >> xx >> xy >> xtheta;
      }
      else if (i % 3 == 0){
        // Calculate and save sigma (D-optimality)
        Eigen::MatrixXd covariance_temp(3, 3);
        covariance_temp << xx, xy, xtheta,
                           xy, yy, ytheta,
                           xtheta, ytheta, thetatheta;
        Eigen::VectorXcd eivals = covariance_temp.eigenvalues();
        double sum_of_logs = log(eivals[0].real()) +
                             log(eivals[1].real()) +
                             log(eivals[2].real());
        double sigma_temp = exp(1.0 / 3.0 * sum_of_logs);
        uncertainty_vec.push_back(sigma_temp);

        // Get new values
        std::stringstream ss;
        ss << line;
        ss >> xx >> xy >> xtheta;
      }
      else if (i % 3 == 1){
        // Get new values
        std::stringstream ss;
        ss << line;
        ss >> xy >> yy >> ytheta;
      }
      else if (i % 3 == 2){
        // Get new values
        std::stringstream ss;
        ss << line;
        ss >> xtheta >> ytheta >> thetatheta;
      }

      i += 1;
    }
  }
  else {
    ROS_INFO("Failed to open file!");
  }

  // Print node number
  std::cout << "Node number: " << uncertainty_vec.size() << std::endl;

  // Plot results
  plotty::plot(uncertainty_vec);
  plotty::xlabel("# of nodes");
  plotty::ylabel("sigma");
  plotty::show();

  return 0;
}
