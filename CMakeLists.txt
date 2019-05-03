cmake_minimum_required(VERSION 2.8.3)
project(crowdbot_active_slam)

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_compile_options(-std=c++11)


# let cmake find OpenMP and set some variables
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBRARIES gomp)
endif()

# csm is too old for catkin_simple
include(FindPkgConfig)
pkg_check_modules(csm REQUIRED csm)
# csm libraries are installed in /opt/ros/kinetic , how to find them otherwise?
link_directories(${csm_LIBRARY_DIRS})
# this adds /opt/ros/kinetic to the library directories, which might cause problems
# as long as the catkin config extends /opt/ros/kinetic, should be ok.


# Libraries
cs_add_library(pedestrian src/pedestrian.cpp)
cs_add_library(utils src/utils.cpp)
cs_add_library(ObjectDetector src/object_detector.cpp)
cs_add_library(TrackedObject src/tracked_object.cpp)
cs_add_library(KalmanFilter src/kalman_filter.cpp)
cs_add_library(graph_optimisation_lib src/graph_optimisation.cpp)
cs_add_library(static_scan_extractor_lib src/static_scan_extractor.cpp)
cs_add_library(graph_optimisation_nodelet src/graph_optimisation_nodelet.cpp)
cs_add_library(static_scan_extractor_nodelet src/static_scan_extractor_nodelet.cpp)
target_link_libraries(static_scan_extractor_lib ${csm_LIBRARIES} ObjectDetector TrackedObject KalmanFilter  )
target_link_libraries(graph_optimisation_lib ${csm_LIBRARIES} )
target_link_libraries(graph_optimisation_nodelet graph_optimisation_lib)
target_link_libraries(static_scan_extractor_nodelet  static_scan_extractor_lib)


# Executables
cs_add_executable(graph_optimisation src/graph_optimisation.cpp)
cs_add_executable(frontier_exploration src/frontier_exploration.cpp)
cs_add_executable(decision_maker src/decision_maker.cpp)
cs_add_executable(combined_laser_scans src/combined_laser_scans.cpp)
cs_add_executable(pedestrian_simulator src/pedestrian_simulator.cpp)
cs_add_executable(static_scan_extractor src/static_scan_extractor.cpp)
target_link_libraries(graph_optimisation graph_optimisation_lib)
target_link_libraries(combined_laser_scans)
target_link_libraries(pedestrian_simulator pedestrian utils)
target_link_libraries(static_scan_extractor static_scan_extractor_lib  )

# test scripts
cs_add_executable(plot_SDF test_results/scripts/plot_SDF.cpp)
cs_add_executable(get_SDF test_results/scripts/get_SDF.cpp)
cs_add_executable(calculate_SDF_error test_results/scripts/calculate_SDF_error.cpp)
cs_add_executable(calculate_map_error test_results/scripts/calculate_map_error.cpp)
cs_add_executable(plot_cov_along_path test_results/scripts/plot_cov_along_path.cpp)
cs_add_executable(pub_map_difference test_results/scripts/pub_map_difference.cpp)
cs_add_executable(plot_SDF_diff test_results/scripts/plot_SDF_diff.cpp)
target_link_libraries(get_SDF gomp)
target_compile_options(get_SDF PRIVATE ${OpenMP_FLAGS})

# add dependencies to make sure that srv files are generated before used
add_dependencies(frontier_exploration crowdbot_active_slam_generate_messages_cpp )
add_dependencies(decision_maker crowdbot_active_slam_generate_messages_cpp )


cs_install()
cs_export()

