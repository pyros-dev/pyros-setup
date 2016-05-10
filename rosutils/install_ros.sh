#!/usr/bin/env bash

set -x
# Add ROS repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
# Install and initialize rosdep
sudo apt-get install python-rosdep -y
sudo `which rosdep` init
rosdep update
