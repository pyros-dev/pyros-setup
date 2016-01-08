Pyros-setup
===========

.. image:: https://travis-ci.org/asmodehn/pyros-setup.svg?branch=indigo
    :target: https://travis-ci.org/asmodehn/pyros-setup

Toolsuite for running ROS environments directly from python code, without any specific requirements outside of usual python.

This WILL BE a pure python package, to be installed in your system, in order to allow easy ROS access from your python environment. 

To install it
::

sudo pip install -i https://testpypi.python.org/pypi pyros_setup

Basically it allows you to do this
::


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


A ROS package WILL BE also provided ( should BECOME a Third Party Release to minimize changes between the two versions )
This package is only useful if you use the extra functionnalities ( like get_master ) even if your ROS system is already loaded
::

import pyros_setup

try:
    import rospy
    import rosgraph
except ImportError:  # if ROS environment is not setup, we emulate it.
    pyros_setup = pyros_setup.delayed_import_auto(base_path=os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..'))
    import rospy
    import rosgraph


roscore_process = None
master = None
launch = None


def setup_module():
    global master
    global roscore_process
    master, roscore_process = pyros_setup.get_master()
    assert master.is_online()


