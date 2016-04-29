#!/usr/bin/env bash
CI_ROS_DISTRO=$1
source /opt/ros/$CI_ROS_DISTRO/setup.bash
mkdir build
cd build

echo $PYTHONPATH
cmake .. -DCMAKE_INSTALL_PREFIX=./install

echo $PYTHONPATH
make -j1
