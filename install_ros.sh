#!/bin/bash
# Apache License 2.0
# Copyright (c) 2020, ROBOTIS CO., LTD.
# Modified by: Esat Sefa Bayhan
# Original File: https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros2_foxy.sh

echo ""
echo "[Set the target ROS version and name of colcon workspace]"
name_ros_version=${name_ros_version:="foxy"}
path_colcon_workspace=${path_colcon_workspace:="/workspace/avai"}
echo "[Note] OS version         >>> Ubuntu 20.04 (Focal Fossa)"
echo "[Note] Target ROS version >>> ROS 2 Foxy Fitzroy"
echo "[Note] Colcon workspace   >>> $path_colcon_workspace"

echo "[Set Locale]"
sudo apt update
sudo apt-get install -yq locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[Setup Sources]"
sudo rm -rf /var/lib/apt/lists/* && sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null'

echo "[Install ROS 2 packages]"
sudo apt-get update
sudo apt-get install -yq ros-$name_ros_version-desktop

echo "[Environment setup]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt-get install -yq \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool \
    git

echo "[Install additional tools]"
source ~/.bashrc
sudo apt-get install -yq graphicsmagick-libmagick-dev-compat
sudo apt-get install -yq \
    ros-foxy-gazebo-* \
    ros-foxy-cartographer \
    ros-foxy-cartographer-ros \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-dynamixel-sdk \
    ros-foxy-turtlebot3-msgs \
    ros-foxy-turtlebot3 \
    python3-rosdep2

echo "[Set the ROS evironment]"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"

sh -c "echo \"alias cw='cd $path_colcon_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd $path_colcon_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cb='cd $path_colcon_workspace && colcon build --symlink-install && source ~/.bashrc'\" >> ~/.bashrc"

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source $path_colcon_workspace/install/local_setup.bash\" >> ~/.bashrc"

sh -c "echo \"export ROS_DOMAIN_ID=30\" >> ~/.bashrc"