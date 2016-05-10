#!/usr/bin/env bash

CI_ROS_DISTRO=$1

# Use rosdep to install current package dependencies
rosdep install --default-yes --from-paths $PKG_PATH --rosdistro $CI_ROS_DISTRO

source /opt/ros/$CI_ROS_DISTRO/setup.bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=./install
make -j1
cd ..
