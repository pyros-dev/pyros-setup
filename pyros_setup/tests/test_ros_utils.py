from __future__ import absolute_import

# Adding current package repository in order to be able to import it (if started with python cli)
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

# importing current package
import pyros_setup

# importing nose
import nose


def test_roscore_started():
    try:
        import rospy
    except ImportError:
        global pyros_setup
        pyros_setup = pyros_setup.delayed_import()

    master = pyros_setup.ROS_Master()
    assert master.is_online()


def test_roslaunch_started():
    try:
        import roslaunch
    except ImportError:
        pyros_setup.delayed_import()  # you do the setup as expected by ROS
        import roslaunch

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    assert launch.started


def test_rosnode_started():
    try:
        import roslaunch
    except ImportError:
        pyros_setup.delayed_import()  # you do the setup as expected by ROS
        import roslaunch

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    assert launch.started

    echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='echo')
    echo_process = launch.launch(echo_node)
    assert echo_process.is_alive()

    # TODO : assert node is up and node_init finished ( to avoid exception )

if __name__ == '__main__':
    # forcing nose run from python call
    nose.runmodule()