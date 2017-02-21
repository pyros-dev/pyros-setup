#!/usr/bin/env python
from __future__ import absolute_import

import sys
import os
import pytest

@pytest.fixture
def cmdopt(request):
    return request.config.getoption("--distro")


@pytest.fixture
def setup():
    # We make sure sys.modules is clear of our test modules for our test here
    for m in [mk for mk in sys.modules.keys() if mk.startswith('rospy')]:
        sys.modules.pop(m, None)
    for m in [mk for mk in sys.modules.keys() if mk.startswith('pyros_setup')]:
        sys.modules.pop('pyros_setup', None)


def test_rospy_imported_config(setup, cmdopt):
    rospy = None

    try:
        import rospy  # this will fail (see setup())
    except ImportError:
        # Proper Way of handling Import Error with pyros-setup

        # Careful this might mean the system installed one if you re not working in virtualenv
        import pyros_setup

        # no need to change package settings here, but we still need to call the configuration step for the import...
        setup = pyros_setup.configurable_import()

        # Loading instance configuration from default file
        setup.configure()

        if cmdopt:
            # dynamically replacing the configured distro we got from default config
            # with the one specified interactively for this test, if present.
            setup.configure({'DISTRO': cmdopt})
            # We need to allow this to fail if the instance configuration is not setup properly (no option),
            # this way the user can debug and fix his configuration.

        setup.activate()  # you do the setup as expected by ROS

        try:
            # we now have access to all imported content (directly or through redirection _PyrosSetup class)
            assert hasattr(pyros_setup, 'configurable_import')
        except ImportError:
            assert False

        import rospy

    assert rospy is not None



