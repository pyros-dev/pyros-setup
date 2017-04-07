#!/usr/bin/env python
from __future__ import absolute_import

import sys
import os
import pytest


@pytest.fixture
def setup():
    # We make sure sys.modules is clear of our test modules for our test here
    for m in [mk for mk in sys.modules.keys() if mk.startswith('rospy')]:
        sys.modules.pop(m, None)
    for m in [mk for mk in sys.modules.keys() if mk.startswith('pyros_setup')]:
        sys.modules.pop('pyros_setup', None)



pytestmark = pytest.mark.skipif(
    not os.path.exists('/opt/ros/indigo'),
    reason="requires ROS indigo distro"
)


def test_rospy_import(setup):
    rospy = None

    try:
        import rospy  # this will fail (see setup())
    except ImportError:
        # Proper Way of handling Import Error with pyros-setup

        # Careful this might mean the system installed one if you re not working in virtualenv
        import pyros_setup.indigo

        # Loading default configuration
        pyros_setup.indigo.configure({'WORKSPACES:[]'})

        # Do the setup as expected by ROS
        pyros_setup.indigo.activate()

        import rospy

    assert rospy is not None
    assert rospy.__file__ == '/opt/ros/indigo/lib/python2.7/dist-packages/rospy/__init__.pyc'


if __name__ == '__main__':
    pytest.main([
        '-s',
        'test_indigo.py'
    ])
