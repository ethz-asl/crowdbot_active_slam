cmake_minimum_required(VERSION 2.8.3)
project(crowdbot_active_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs)
find_package(GTSAM REQUIRED)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp std_msgs geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(graph_optimisation src/graph_optimisation.cpp)
target_link_libraries(graph_optimisation gtsam ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
