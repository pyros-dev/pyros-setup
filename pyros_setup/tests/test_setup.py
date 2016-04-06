#!/usr/bin/env python
from __future__ import absolute_import

# Adding current package repository in order to be able to import it
# if started with python cli, since relative import from non-package are forbidden
import sys
import os

# importing nose
import nose


def setup_function():
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


def teardown_function():
    pass


@nose.tools.with_setup(setup_function, teardown_function)
def test_rospy_imported_config():
    rospy = None

    # Careful this might mean the system installed one if you re not working in virtualenv
    import pyros_setup
    try:
        import rospy  # this will fail unless
    except ImportError:
        # Proper New way
        # TODO : make this work with module/import, instead of class/instantiation
        pyros_setup.configurable_import(instance_relative_config=False)\
            .configure('tests/testing.cfg')\
            .activate()  # you do the setup as expected by ROS
        import rospy

    assert rospy is not None

    try:
        # we now have access to all imported content (directly or through redirection _PyrosSetup class)
        assert hasattr(pyros_setup, 'configurable_import')
    except ImportError:
        assert False


@nose.tools.with_setup(setup_function, teardown_function)
def test_rospy_imported():
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


@nose.tools.with_setup(setup_function, teardown_function)
def test_rospy_imported_auto():
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


# TODO : fix this, we need each test to run in an isolated environment(=> forced multiprocess)?
# currently you need to manually comment all test you do not want to run...
if __name__ == '__main__':
    # forcing nose run from python call
    nose.runmodule()
