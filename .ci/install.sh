#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install"

MY_PATH=`pwd`

sudo apt-get -y install git

echo "clone uav_core"
cd
git clone https://github.com/ctu-mrs/uav_core.git
cd uav_core
./installation/install.sh

echo "clone simulation"
cd
git clone https://github.com/ctu-mrs/simulation.git
cd simulation
./installation/install.sh

echo "clone uvdar_core"
cd
git clone https://github.com/ctu-mrs/uvdar_core.git

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/uav_core
ln -s ~/simulation
ln -s ~/uvdar_core
ln -s "$MY_PATH" uvdar_gazebo_plugin
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "install ended"
