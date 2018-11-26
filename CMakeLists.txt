cmake_minimum_required(VERSION 2.8.3)
project(crowdbot_active_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_msgs
  nav_msgs
  cmake_modules
  message_generation
  pcl_ros)

find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plotty REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(glog_catkin REQUIRED)
pkg_check_modules(csm REQUIRED csm)
pkg_check_modules(SBPL REQUIRED sbpl)
find_package(OpenCV REQUIRED)

add_compile_options(-std=c++11)

# let cmake find OpenMP and set some variables
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBRARIES gomp)
endif()

add_service_files(
  FILES
  get_frontier_list.srv
  get_map.srv
  service_call.srv
  utility_calc.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp roslib tf std_msgs sensor_msgs geometry_msgs gazebo_msgs pcl_ros
DEPENDS EIGEN3 plotty OpenMP)

include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIRS} ${SBPL_INCLUDE_DIRS}
                            ${plotty_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS}
                            ${GTSAM_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}
                            ${pcl_ros_INCLUDE_DIRS} ${glog_catkin_INCLUDE_DIRS})

link_directories(${csm_LIBRARY_DIRS} ${SBPL_LIBRARY_DIRS})

add_executable(graph_optimisation src/graph_optimisation.cpp)
add_executable(frontier_exploration src/frontier_exploration.cpp)
add_executable(decision_maker src/decision_maker.cpp)
add_executable(combined_laser_scans src/combined_laser_scans.cpp)

target_link_libraries(graph_optimisation gtsam
                      ${catkin_LIBRARIES} ${csm_LIBRARIES} ${EIGEN3_LIBRARIES} ${GTSAM_LIBRARIES} ${libpointmatcher_LIBRARIES})
target_link_libraries(frontier_exploration ${catkin_LIBRARIES})
target_link_libraries(decision_maker ${catkin_LIBRARIES} ${SBPL_LIBRARIES})
target_link_libraries(combined_laser_scans ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} ${glog_catkin_LIBRARIES})

# add dependencies to make sure that srv files are generated before used
add_dependencies(graph_optimisation crowdbot_active_slam_generate_messages_cpp)
add_dependencies(frontier_exploration crowdbot_active_slam_generate_messages_cpp)
add_dependencies(decision_maker crowdbot_active_slam_generate_messages_cpp)

# test scripts
add_executable(plot_SDF test_results/scripts/plot_SDF.cpp)
add_executable(get_SDF test_results/scripts/get_SDF.cpp)
add_executable(calculate_SDF_error test_results/scripts/calculate_SDF_error.cpp)
add_executable(calculate_map_error test_results/scripts/calculate_map_error.cpp)
add_executable(plot_cov_along_path test_results/scripts/plot_cov_along_path.cpp)

target_compile_options(get_SDF PRIVATE ${OpenMP_FLAGS})
target_link_libraries(get_SDF ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenMP_LIBRARIES})
target_link_libraries(calculate_SDF_error ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(calculate_map_error ${catkin_LIBRARIES})
target_link_libraries(plot_cov_along_path ${catkin_LIBRARIES} ${plotty_LIBRARIES})
target_link_libraries(plot_SDF ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBRARIES})

install(DIRECTORY
  launch
  matlab
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
