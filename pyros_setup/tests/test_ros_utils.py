from __future__ import absolute_import

# Adding current package repository in order to be able to import it (if started with python cli)
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
import time

# importing current package
import pyros_setup

# importing nose
import nose


class timeout(object):
    """
    Small useful timeout class
    """
    def __init__(self, seconds):
        self.seconds = seconds

    def __enter__(self):
        self.die_after = time.time() + self.seconds
        return self

    def __exit__(self, type, value, traceback):
        pass

    @property
    def timed_out(self):
        return time.time() > self.die_after


def test_roscore_started():
    try:
        import rospy
    except ImportError:
        global pyros_setup
        pyros_setup = pyros_setup.delayed_import()

    master = pyros_setup.get_master()
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
        import rospy
        import rosnode
        import roslaunch
    except ImportError:
        global pyros_setup
        pyros_setup = pyros_setup.delayed_import()  # you do the setup as expected by ROS
        import rospy
        import rosnode
        import roslaunch

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    assert launch.started

    echo_node = roslaunch.core.Node('pyros_test', 'echo.py', name='echo')
    echo_process = launch.launch(echo_node)
    assert echo_process.is_alive()

    node_api = None
    with timeout(5) as t:
        while not t.timed_out and node_api is None:
            node_api = rosnode.get_api_uri(pyros_setup.get_master(), 'echo')
    assert node_api is not None

if __name__ == '__main__':
    # forcing nose run from python call
    nose.runmodule()