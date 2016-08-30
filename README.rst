Pyros-setup
===========

.. image:: https://travis-ci.org/asmodehn/pyros-setup.svg?branch=master
    :target: https://travis-ci.org/asmodehn/pyros-setup

Toolsuite for running ROS environments directly from python code, without any specific requirements outside of usual python.

This is a pure python package, to be installed in your system, in order to allow easy ROS access from your python environment.

HowTo install
^^^^^^^^^^^^^

To install it::

  pip install pyros_setup

To run the self tests, using entry_points defined in setup.py::

  pyros_setup --pytest

OR using the python package directly::

  python -m pyros_setup --pytest

OR using pytest specifically, optionally specifying the distro you want to use ::

  py.test -s --pyargs pyros_setup --distro=indigo

OR via tox to test multiple env at once (with only one ROS distro) ::

  tox -- --distro=indigo

It can also be used from source inside a catkin workspace in the same way.
The workspace act as a virtual environment (using https://github.com/asmodehn/catkin_pip).
This is useful for development along with ROS packages::

  $ catkin_make
  $ source devel/setup.bash
  $ python -m pyros_setup
  $ pyros_setup
  $ py.test -s --pyargs pyros_setup --distro=indigo


HowTo code
^^^^^^^^^^

Basically it allows you to do this::

  import pyros_setup
  try:
      import rospy
      import roslaunch
      import rosgraph
      import rosnode
  except ImportError:  # if ROS environment is not setup, we emulate it.
      pyros_setup.configurable_import().configure('mysetup.cfg').activate()  # this will use mysetup.cfg from pyros-setup instance folder
      import rospy
      import roslaunch
      import rosgraph
      import rosnode

With mysetup.cfg in pyros-setup instance folder containing::

  import os
  WORKSPACES=[os.path.join('home', 'user', 'ROS', 'workspace', 'devel')]
  DISTRO='indigo'


Note: If you know any easier / less tricky / more pythonic way of handling configurable dynamic imports, let me know!

HowTo deploy
^^^^^^^^^^^^

If you want to use pyros-setup, you should use the pip package, since the whole point is to provide access to ROS from pure python environment.
This is now possible thanks to [catkin_pip](https://github.com/asmodehn/catkin_pip)


Remarks
^^^^^^^

Although it would technically be possible to build a ROS package from this source, this will NOT be done.
The catkin build system is only here to help having pyros-setup in a source workspace while developing on it.
When using ROS directly this package is not needed, and having it installed among ROS packages would cause much user confusion when importing packages.

Troubleshooting
^^^^^^^^^^^^^^^

1. Wrong ROS Distro

  If you run self tests like this ::

    $ pyros_setup --pytest

  You might get ::

    ========================================================================================================= test session starts =========================================================================================================
    platform linux2 -- Python 2.7.6, pytest-3.0.1, py-1.4.31, pluggy-0.3.1
    rootdir: /home/alexv/Projects/pyros-setup, inifile:
    collected 1 items

    pyros_setup/tests/test_setup.py WARNING:root:Error detected while importing ROS python modules. Attempting fix via ROS setup emulation...
    WARNING:root: => Pyros_setup v0.1.99 Emulating ROS setup now for distro kinetic and workspaces ()
    WARNING:root:Configured workspace /opt/ros/kinetic not found. Please double check your configuration. Skipping...
    WARNING:root: => ROS setup emulation done.
    [...]
    E       ImportError: No module named rospy

    /usr/lib/python2.7/importlib/__init__.py:37: ImportError
    ====================================================================================================== 1 failed in 0.02 seconds =======================================================================================================

  This is what happens when the configuration (default) tries to use a ROS distro you do NOT have on your system
  rospy just cannot be found.

  To try detecting multiple ROS distro on your system you can pass the `--distro` option.

2. VirtualEnv not using system packages

  If, upon running test with ::

    $ pyros_setup --pytest --distro=indigo

  You get ::

    ========================================================================================================= test session starts =========================================================================================================
    platform linux2 -- Python 2.7.6, pytest-3.0.1, py-1.4.31, pluggy-0.3.1
    rootdir: /home/alexv/Projects/pyros-setup, inifile:
    collected 1 items

    pyros_setup/tests/test_setup.py WARNING:root:Error detected while importing ROS python modules. Attempting fix via ROS setup emulation...
    WARNING:root: => Pyros_setup v0.1.99 Emulating ROS setup now for distro indigo and workspaces ()
    WARNING:root:Prepending path /opt/ros/indigo to CMAKE_PREFIX_PATH
    WARNING:root:Prepending path /opt/ros/indigo/bin to PATH
    WARNING:root:Prepending path /opt/ros/indigo/lib to LD_LIBRARY_PATH
    WARNING:root:Prepending path /opt/ros/indigo/lib/pkgconfig to PKG_CONFIG_PATH
    WARNING:root:Prepending path /opt/ros/indigo/lib/python2.7/dist-packages to PYTHONPATH
    WARNING:root: => ROS setup emulation done.
    ERROR:root:importlib.import_module(rospy) FAILED : No module named yaml
    ERROR:root:Make sure you have installed the yaml python package
    [...]
    E   ImportError: No module named yaml

    /opt/ros/indigo/lib/python2.7/dist-packages/rospy/client.py:47: ImportError
    ====================================================================================================== 1 failed in 0.03 seconds =======================================================================================================

  This means your virtualenv cannot access system (and ROS) packages.

  This is easily fixed by removing `<virtualenv_dir>/lib/pythonX.Y/no-global-site-packages.txt` to allow your virtualenv to also include system (and ROS) packages.
