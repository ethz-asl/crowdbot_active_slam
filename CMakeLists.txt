cmake_minimum_required(VERSION 2.8.3)
project(crowdbot_active_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules)

find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_check_modules(csm REQUIRED csm)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp tf std_msgs sensor_msgs geometry_msgs
DEPENDS Eigen3)

include_directories(include ${catkin_INCLUDE_DIRS} ${csm_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
link_directories(${csm_LIBRARY_DIRS})

add_executable(graph_optimisation src/graph_optimisation.cpp)
target_link_libraries(graph_optimisation gtsam
                      ${catkin_LIBRARIES} ${csm_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
