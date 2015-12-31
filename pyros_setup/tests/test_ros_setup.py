from __future__ import absolute_import

# Adding current package repository in order to be able to import it (if started with python cli)
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

# importing current package
import pyros_setup

# importing nose
import nose


def test_rospy_imported():
    rospy = None
    try:
        import rospy  # this will fail unless
    except ImportError:
        pyros_setup.ROS_emulate_setup()  # you do the setup as expected by ROS
        import rospy

    assert rospy is not None


if __name__ == '__main__':
    # forcing nose run from python call
    nose.runmodule()