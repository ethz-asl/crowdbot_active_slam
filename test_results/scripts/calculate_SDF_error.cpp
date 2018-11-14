#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>

Eigen::MatrixXf load_SDF(std::string sdf_path){
  // Open file
  std::ifstream sdf_file(sdf_path.c_str());

  std::vector<std::vector<float>> pre_mat;
  std::string line;
  if (sdf_file.is_open()){
    while (std::getline(sdf_file, line)){
      std::stringstream ss;
      ss << line;
      float dist;
      std::vector<float> temp_vec;
      while (ss >> dist){
        temp_vec.push_back(dist);
      }
      pre_mat.push_back(temp_vec);
    }
  }
  else {
    ROS_INFO("failed");
  }

  int width = pre_mat.size();
  int height = pre_mat[0].size();
  Eigen::MatrixXf sdf_mat(width, height);
  for (int i = 0; i < width; i++){
    for (int j = 0; j < height; j++){
      sdf_mat(i, j) = pre_mat[i][j];
    }
  }
  return sdf_mat;
}


int main(int argc, char **argv)
{
  // Get path and file name
  std::string package_path = ros::package::getPath("crowdbot_active_slam");
  std::string test_sdf_path = package_path + "/" + argv[1];
  std::string ref_sdf_path = package_path + "/" + argv[2];

  Eigen::MatrixXf test_sdf_mat = load_SDF(test_sdf_path);
  Eigen::MatrixXf ref_sdf_mat = load_SDF(ref_sdf_path);

  // Calculate score
  double sdf_score = 0;

  for (int i = 0; i < test_sdf_mat.rows(); i++){
    for (int j = 0; j < test_sdf_mat.cols(); j++){
      sdf_score += pow(test_sdf_mat(i, j) - ref_sdf_mat(i, j), 2);
    }
  }

  std::cout << "sdf_score: " << sdf_score << std::endl;
  // std::cout << "Number of known cells: " << i << std::endl;
  // std::cout << "Number of known test cells, while unknown in ref map: " << j << std::endl;
  // std::cout << "score for known test cells, while unkown in ref map: " << over_ref_score << std::endl;
  // std::cout << "Number of known ref cells, while unknown in test map: " << k << std::endl;
  return 0;
}
