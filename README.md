# crowdbot_active_slam

This repository contains tools for active SLAM in crowded environments. It contains code that has been tested and used with a pioneer in simulation, a real pepper robot and a real turtlebot3-pi.

## Installation

1. Install most needed ROS packages by executing:

  ```bash
  bash install.sh
  ```

  This script will also remove a move_base installation, if there is one installed, as there has been found a bug, which is fixed in this [navigation](https://github.com/dmammolo/navigation) fork.

2. Clone all packages listed in **Needed Packages**.

  - Make sure to checkout on devel branch for [naoqi_driver](https://github.com/danieldugas/naoqi_driver).
  - Make sure to checkout on devel branch for [asl_pepper](https://github.com/ethz-asl/asl_pepper).
  - Make sure to checkout on kinetic-devel branch for [navigation](https://github.com/dmammolo/navigation).

3. Install GTSAM library following the steps here [GTSAM 3.2.1](https://borg.cc.gatech.edu/download.html). (Only version 3.2.1 has been tested)

4. Make sure that you have an [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) installation.

5. For building all packages either remove certain packages or only build the bellow listed ones:

  ```bash
  catkin build sick_scan gazebo_ros_2Dmap_plugin pioneer_description naoqi_driver eigen_catkin glog_catkin eigen_catkin plotty asl_pepper_basic_functions asl_pepper_joystick
  ```

6. Finally build this package and the fixed move_base package. We recommend to build it in Release mode to assure online behaviour:

  ```bash
  catkin build crowdbot_active_slam move_base --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

### Needed Packages

- [gazebo_ros_2Dmap_plugin](https://github.com/dmammolo/gazebo_ros_2Dmap_plugin)
- [pioneer_description](https://github.com/dmammolo/pioneer_description)
- [navigation](https://github.com/dmammolo/navigation)
- [naoqi_driver](https://github.com/danieldugas/naoqi_driver)
- [asl_pepper](https://github.com/ethz-asl/asl_pepper)
- [eigen_catkin](https://github.com/ethz-asl/eigen_catkin)
- [catkin_simple](https://github.com/catkin/catkin_simple)
- [plotty](https://github.com/ethz-asl/plotty)
- [glog_catkin](https://github.com/ethz-asl/glog_catkin)
- [sick_scan](https://github.com/SICKAG/sick_scan)

If you plan to use turtlebot3, follow the instructions on <http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/>.

## Further Usage Information

### Bash Scripts for Launching Nodes

This package consists of multiple nodes, which interact with each other. To facilitate the launching of these nodes, bash scripts are used to start up different scenarios.

- All run_pioneer_* scripts use the gazebo simulation

  - **run_pioneer_SLAM.sh** starts the SLAM algorithm
  - **run_pioneer_exploration.sh** starts the active SLAM framework for static environments
  - **run_pioneer_exploration_dyn_sim.sh** starts additionally the people(pedestrian) simulator and uses the detection and tracking of moving people

- run_pepper_* scripts are intended to be used in reality with the pepper robot

  - **run_pepper_SLAM.sh** starts again the SLAM algorithm
  - **run_pepper_SLAM_exploration.sh** starts the active SLAM framework in static environments mode
  - **run_pepper_SLAM_exploration_dyn.sh** starts the active SLAM framework with the detection and tracking of moving people

If you further want to adapt or change parameters like exploration strategies, simulation worlds, scan matcher parameters, etc. refer to the corresponding launch files. In general the main SLAM file is called **graph_optimisation** and contains Front- and Back-End.

### Evaluation

The [test_results](https://github.com/ethz-asl/crowdbot_active_slam/tree/devel/test_result) folder contains evaluation scripts for exploration runs. The necessary data is automatically saved during exploration and is saved in this folder.

### Miscellaneous

This package contains a nodelet implementation, but is currently not running without crashing. In future one might solve the issue.

The [gazebo_ros_2Dmap_plugin](https://github.com/dmammolo/gazebo_ros_2Dmap_plugin) is a plugin, which can be used to extract the groundtruth map of asimulation world.

## Known issues

- GTSAM 3.2.1 boost issue. If you have an issue building or using GTSAM, you may need to adapt some files:

  - gtsam/inference/Ordering.cpp, add this line

    ```c++
    #include <boost/serialization/serialization.hpp>
    ```

  - gtsam/base/tests/testFastContainers.cpp, add this line

    ```c++
    #include <boost/serialization/serialization.hpp>
    ```

- Installing cartographer ros forces installing protobuf >v3.4.1\. This is not compatible with gazebo_ros and can cause problems when building this packages used here, e.g. gazebo_ros_2Dmap_plugin.

- If you want to move peppers head with the joystick you will need to add the naoqi SDK to your python path.

(Last update: 22.03.2019)
