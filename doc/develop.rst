Pyros-setup
===========

Development guide
-----------------

Pyros-setup is a pure python to ROS interface.
It can dynamically discover a ROS setup, and a user can develop normal python packages that interract with ROS, without needing to know anything about ROS.
Only using pyros from its pip package should be enough.

Developing such interfacing code needs to be done carefully : 

- pyros-setup needs to be fully functional as a pip package. dynamically discovering ROS dependencies, and providing them to python users.
=> a pip package is provided

- pyros-setup needs to be usable in ROS as a normal ros package so that ROS developers can also leverage it in their development.
=> a ROS package is provided

Both package should not conflict on a user system.
The ROS package is used (if installed) when ROS is setup (setup.bash sourced).
The pip package is used (if installed) when ROS is not setup (setup.bash not sourced).

Development Environment Setup
-----------------------------

To work on lowlevel package like pyros-setup and guarantee close compatibility both with ros and python, it is better to install only a minimal ROS system.
And to be able to quickly learn the code and identify issues in underlying dependencies, sources are always better.

ROS From Source (ubuntu)
--------------------
 Ref : http://wiki.ros.org/indigo/Installation/Source

Setting up ROS package repositories (maybe not needed if we get everything from source. better to skip this to make sure we know when rosdep wants to grab ROS packages)
  $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Getting ROS dependencies from source
  $ sudo apt-get update
  $ sudo apt-get install build-essential

  $ git clone https://github.com/vcstools/wstool.git
  $ git clone https://github.com/vcstools/rosinstall.git
  $ git clone https://github.com/ros-infrastructure/rosinstall_generator.git
  $ git clone https://github.com/ros-infrastructure/rosdep
  $ cd rosdep
  $ source setup.sh

  $ sudo rosdep init
  $ rosdep update

Getting minimal ROS
  $ mkdir ~/ros_catkin_ws
  $ cd ~/ros_catkin_ws
  $ rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --tar > indigo-ros_comm-wet.rosinstall
  $ wstool init -j8 src indigo-ros_comm-wet.rosinstall
  $ rosdep install --from-paths src --ignore-src --rosdistro indigo -y
  $ ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Debug

Updating minimal ROS
  $ mv -i indigo-ros_comm-wet.rosinstall indigo-ros_comm-wet.rosinstall.old
  $ rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --tar > indigo-ros_comm-wet.rosinstall
  $ diff -u indigo-ros_comm-wet.rosinstall indigo-ros_comm-wet.rosinstall.old
  $ wstool merge -t src indigo-ros_comm-wet.rosinstall
  $ wstool update -t src
  $ ./src/catkin/bin/catkin_make_isolated --install

Sourcing ROS setup (and dropping into usual ROS setup)
  $ source ~/ros_catkin_ws/install_isolated/setup.bash

