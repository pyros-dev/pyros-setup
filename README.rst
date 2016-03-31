Pyros-setup
===========

.. image:: https://travis-ci.org/asmodehn/pyros-setup.svg?branch=indigo
    :target: https://travis-ci.org/asmodehn/pyros-setup

Toolsuite for running ROS environments directly from python code, without any specific requirements outside of usual python.

This is a pure python package, to be installed in your system, in order to allow easy ROS access from your python environment.

To install it::

  sudo pip install -i https://testpypi.python.org/pypi pyros_setup

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

This allows you to use your own package as a normal python package, with python workflow (example using virtualenvwrapper)::

  $ mkvirtualenv my_package_venv --system-site-packages
  (my_package_venv)$ pip install -r requirements.txt
  (my_package_venv)$ python -m my_package
  (my_package_venv)$ nosetests
  (my_package_venv)$ deactivate
  $

OR using the python workflow from inside a catkin workspace::

  $ source /opt/ros/indigo/setup.bash
  $ cd existing_catkin_ws
  $ catkin_make
  $ source devel/setup.bash
  $ python -m my_package
  $ nosetests

A ROS package WILL NOT be provided, since there is no simple and clean way to turn a pure python package into a catkin package.
If you want to depend on pyros-setup, you should use the rosdep pip dependency mechanism.

Note: If you know any easier / less tricky / more pythonic way of handling dynamic imports, let me know!
