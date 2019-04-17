# Install all necessary ros packages
sudo apt-get install -y ros-kinetic-csm
sudo apt-get install -y ros-kinetic-teb-local-planner
sudo apt-get install -y ros-kinetic-grid-map-ros
sudo apt-get install -y ros-kinetic-gazebo-ros-control
sudo apt-get install -y ros-kinetic-diff-drive-controller
sudo apt-get install -y ros-kinetic-costmap-2d
sudo apt-get install -y ros-kinetic-laser-geometry
sudo apt-get install -y ros-kinetic-filters

# Install all pepper dependent packages
sudo apt-get install -y ros-kinetic-driver-base
sudo apt-get install -y ros-kinetic-move-base-msgs
sudo apt-get install -y ros-kinetic-octomap
sudo apt-get install -y ros-kinetic-octomap-msgs
sudo apt-get install -y ros-kinetic-humanoid-msgs
sudo apt-get install -y ros-kinetic-humanoid-nav-msgs
sudo apt-get install -y ros-kinetic-camera-info-manager
sudo apt-get install -y ros-kinetic-camera-info-manager-py
sudo apt-get install -y ros-kinetic-pepper-.*
