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
    # We make sure sys.modules is clear of our test modules for each test we run here
    for m in [mk for mk in sys.modules.keys() if mk.startswith('rospy')]:
        sys.modules.pop(m, None)
    for m in [mk for mk in sys.modules.keys() if mk.startswith('pyros_setup')]:
        sys.modules.pop('pyros_setup', None)

    # prepending because ROS relies on package dirs list in PYTHONPATH and not isolated virtualenvs
    # And we need our current module to be found first.
    current_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    # if not current_path in sys.path:
    sys.path.insert(1, current_path)  # sys.path[0] is always current path as per python spec
    # FIXME : this stack up at every test (to avoid the previous ROs emulated setup to find a different pyros_setup
    # TODO We need a better way...
    # also since we change to py.test the self import thingy is not really what we want to do
    # Maybe we should use python setup.py develop/install instead ?


def test_rospy_imported_config(setup, cmdopt):
    rospy = None

    # Careful this might mean the system installed one if you re not working in virtualenv
    import pyros_setup
    try:
        import rospy  # this will fail unless
    except ImportError:
        # Proper New way

        # no need to change package settings here, but we still need to call the configuration step for the import...
        pyros_setup = pyros_setup.configurable_import()

        # Loading instance configuration
        pyros_setup.configure()

        if cmdopt:
            # dynamically replacing the configured distro we got from default config
            # with the one specified interactively for this test, if present.
            pyros_setup.configure({'DISTRO': cmdopt})
            # We need to allow this to fail if the instance configuration is not setup properly (no option),
            # this way the user can debug and fix his configuration.

        pyros_setup.activate()  # you do the setup as expected by ROS

        import rospy

    assert rospy is not None

    try:
        # we now have access to all imported content (directly or through redirection _PyrosSetup class)
        assert hasattr(pyros_setup, 'configurable_import')
    except ImportError:
        assert False

