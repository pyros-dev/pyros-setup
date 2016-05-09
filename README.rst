Pyros-setup
===========

.. image:: https://travis-ci.org/asmodehn/pyros-setup.svg?branch=master
    :target: https://travis-ci.org/asmodehn/pyros-setup

Toolsuite for running ROS environments directly from python code, without any specific requirements outside of usual python.

This is a pure python package, to be installed in your system, in order to allow easy ROS access from your python environment.

To install it::

  sudo pip install -i https://testpypi.python.org/pypi pyros_setup

To run the self tests, using entry_points defined in setup.py::

  pyros_setup

OR using the python package directly::

  python -m pyros_setup

OR using nosetests specifically::

  nosetests pyros_setup

It can also be used from source inside a catkin workspace in the same way.
The workspace act as a virtual environment (using https://github.com/asmodehn/catkin_pure_python).
This is useful for development along with ROS packages::

  $ catkin_make
  $ source devel/setup.bash
  $ python -m pyros_setup
  $ pyros_setup
  $ nosetests pyros_setup




However it is also usable by catkin from source for ease of development and compatibility testing.

Basically it allows you to do this::

  try:
      import rospy
      import roslaunch
      import rosgraph
      import rosnode
  except ImportError:  # if ROS environment is not setup, we emulate it.
      import pyros_setup
      pyros_setup = pyros_setup.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..'))
      import rospy
      import roslaunch
      import rosgraph
      import rosnode


A ROS package WILL NOT be provided, since there is no simple and clean way to turn a pure python package into a catkin package.
If you want to depend on pyros-setup, you should use the rosdep pip dependency mechanism.

Note: If you know any easier / less tricky / more pythonic way of handling dynamic imports, let me know!
