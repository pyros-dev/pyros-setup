Pyros-setup
===========

.. image:: https://travis-ci.org/asmodehn/pyros-setup.svg?branch=master
    :target: https://travis-ci.org/asmodehn/pyros-setup

Toolsuite for running ROS environments directly from python code, without any specific requirements outside of usual python.

This is a pure python package, to be installed in your system, in order to allow easy ROS access from your python environment.

HowTo install
^^^^^^^^^^^^^

To install it::

  pip install pyros_setup

To run the self tests, using entry_points defined in setup.py::

  pyros_setup

OR using the python package directly::

  python -m pyros_setup

OR using nosetests specifically::

  nosetests pyros_setup

It can also be used from source inside a catkin workspace in the same way.
The workspace act as a virtual environment (using https://github.com/asmodehn/catkin_pip).
This is useful for development along with ROS packages::

  $ catkin_make
  $ source devel/setup.bash
  $ python -m pyros_setup
  $ pyros_setup
  $ nosetests pyros_setup


HowTo code
^^^^^^^^^^

Basically it allows you to do this::

  import pyros_setup
  try:
      import rospy
      import roslaunch
      import rosgraph
      import rosnode
  except ImportError:  # if ROS environment is not setup, we emulate it.
      pyros_setup.configurable_import().configure('mysetup.cfg').activate()  # this will use mysetup.cfg from pyros-setup instance folder
      import rospy
      import roslaunch
      import rosgraph
      import rosnode

With mysetup.cfg in pyros-setup instance folder containing::

  import os
  WORKSPACES=[os.path.join('home', 'user', 'ROS', 'workspace', 'devel')]
  DISTRO='indigo'


Note: If you know any easier / less tricky / more pythonic way of handling configurable dynamic imports, let me know!

HowTo deploy
^^^^^^^^^^^^

If you want to use pyros-setup, you should use the pip package, since the whole point is to provide access to ROS from pure python environment.
This is now possible thanks to [catkin_pip](https://github.com/asmodehn/catkin_pip)


Remarks
^^^^^^^

Although it would technically be possible to build a ROS package from this source, this will NOT be done.
The catkin build system is only here to help having pyros-setup in a source workspace while developing on it.
When using ROS directly this package is not needed, and having it installed among ROS packages would cause much user confusion when importing packages.
