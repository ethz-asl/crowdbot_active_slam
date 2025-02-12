#include <iostream>
#include <fstream>
#include <plotty/matplotlibcpp.hpp>

#include <ros/ros.h>
#include <ros/package.h>


void getUncertainty(std::string file_path, std::vector<double>& uncertainty_vec,
                                              std::vector<double>& path_length_vec){
  std::ifstream cov_file(file_path.c_str());
  int i = 0;
  double path_length, xx, xy, xtheta, yy, ytheta, thetatheta;
  std::string line;

  if (cov_file.is_open()){
    while (std::getline(cov_file, line)){
      if (i == 0){
        // Get new values
        std::stringstream ss;
        ss << line;
        ss >> path_length >> xx >> xy >> xtheta;
        path_length_vec.push_back(path_length);
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
        ss >> path_length >> xx >> xy >> xtheta;
        path_length_vec.push_back(path_length);
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
    // Calculate and save last sigma (D-optimality)
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
  }
  else {
    ROS_INFO("Failed to open file!");
  }
}


int main(int argc, char **argv)
{
  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");

std::string general_results_path;
  if (argc == 2){
    general_results_path = package_path + "/" + argv[1] + "/general_results.txt";
  }

  int nr_of_tests = argc;

  std::vector<std::vector<double>> uncertainty_vecs;
  std::vector<std::vector<double>> path_length_vecs;
  std::vector<std::string> paths;
  for (int i = 1; i < nr_of_tests; i++){
    paths.push_back(package_path + "/" + argv[i] + "uncertainty_matrices.txt");

    std::vector<double> uncertainty_vec;
    std::vector<double> path_length_vec;
    getUncertainty(paths[i - 1], uncertainty_vec, path_length_vec);
    uncertainty_vecs.push_back(uncertainty_vec);
    path_length_vecs.push_back(path_length_vec);
  }

  if (argc == 2){
    std::ifstream result_file_in(general_results_path.c_str());
    std::string result_file_string = "";
    std::string line;
    std::string node_number = "Node number";
    if (result_file_in.is_open()){
      while (std::getline(result_file_in, line)){
        if (line.find(node_number) != std::string::npos){
          result_file_string += "Node number: " + std::to_string(uncertainty_vecs[0].size()) + '\n';
        }
        else {
          result_file_string += line + '\n';
        }
      }
      result_file_in.close();
    }
    else {
      ROS_INFO("Could not open general_results(input file).txt!");
    }
    std::ofstream result_file_out(general_results_path.c_str());
    if (result_file_out.is_open()){
      result_file_out << result_file_string;
    }
    else {
      ROS_INFO("Could not open general_results(output file).txt!");
    }
  }

  // Print node number
  std::cout << "Node number: " << uncertainty_vecs[0].size() << std::endl;

  // Plot results
  for (int i = 1; i < nr_of_tests; i++){
    if (i < 6){
      plotty::plot(path_length_vecs[i - 1], uncertainty_vecs[i - 1], "b");
    }
    else if (i < 11){
      plotty::plot(path_length_vecs[i - 1], uncertainty_vecs[i - 1], "r");
    }
    else {
      plotty::plot(path_length_vecs[i - 1], uncertainty_vecs[i - 1], "g");
    }
  }
  plotty::xlabel("path length [m]");
  plotty::ylabel("sigma [1]");
  // plotty::save(package_path + + "/" + argv[1] + "/uncertainy_along_path.png");
  plotty::show();

  return 0;
}
