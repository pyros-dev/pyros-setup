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
        # TODO : make this work with module/import, instead of class/instantiation
        pyros_setup.configurable_import(instance_relative_config=False)\
            .configure('tests/testing_' + cmdopt + '.cfg')\
            .activate()  # you do the setup as expected by ROS
        import rospy

    assert rospy is not None

    try:
        # we now have access to all imported content (directly or through redirection _PyrosSetup class)
        assert hasattr(pyros_setup, 'configurable_import')
    except ImportError:
        assert False


def test_rospy_imported(setup):
    rospy = None

    # Careful this might mean the system installed one if you re not working in virtualenv
    import pyros_setup
    try:
        import rospy  # this will fail unless
    except ImportError:
        # Proper Old bwcompat way
        # sys.modules['pyros_setup'] = pyros_setup.delayed_import()  # you do the setup as expected by ROS
        # But here we just want to replace only local symbol (and avoid replacing all 'import' calls)
        pyros_setup = pyros_setup.delayed_import()  # you do the setup as expected by ROS
        import rospy

    assert rospy is not None

    try:
        # we now have access to all imported content (directly or through redirection _PyrosSetup class)
        assert hasattr(pyros_setup, 'delayed_import')
    except ImportError:
        assert False


def test_rospy_imported_auto(setup):
    rospy = None

    # Careful this might mean the system installed one if you re not working in virtualenv
    import pyros_setup
    try:
        import rospy  # this will fail unless
    except ImportError:
        # Proper way
        sys.modules['pyros_setup'] = pyros_setup.delayed_import_auto()  # you do the setup as expected by ROS
        # But this still works
        # pyros_setup = pyros_setup.delayed_import_auto()  # you do the setup as expected by ROS
        import rospy

    assert rospy is not None

    try:
        # we now have access to all imported content (directly or through redirection _PyrosSetup class)
        assert hasattr(pyros_setup, 'delayed_import_auto')
    except ImportError:
        assert False

