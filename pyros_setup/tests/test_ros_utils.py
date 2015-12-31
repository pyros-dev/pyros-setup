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
    master = pyros_setup.ROS_Master()
    assert master.is_online()


def test_roslaunch_started():
    try:
        import roslaunch
    except ImportError:
        pyros_setup.ROS_emulate_setup()  # you do the setup as expected by ROS
        import roslaunch

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    assert launch.started


if __name__ == '__main__':
    # forcing nose run from python call
    nose.runmodule()