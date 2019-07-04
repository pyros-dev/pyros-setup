Pyros-setup
===========

.. image:: https://travis-ci.org/asmodehn/pyros-setup.svg?branch=master
    :target: https://travis-ci.org/asmodehn/pyros-setup

Toolsuite for using ROS environments directly from python code, without any specific requirements outside of usual python.

This is a pure python package, to be installed in your system, in order to allow easy ROS access from your python environment.

Note : If instead you prefer doing things the other way around, to embed existing python packages in the ROS environment, this is possible thanks to [catkin_pip](https://github.com/asmodehn/catkin_pip)

Prerequisites
^^^^^^^^^^^^^

ROS should be installed on your system.
Example with ROS melodic on Ubuntu Bionic::

  sudo apt install ros-melodic-ros-base

This will setup everything you need for using ROS with python2.7
More information there : http://wiki.ros.org/melodic/Installation/Ubuntu

Then you need to make sure the Python2 virtual environment in which you are working allows access to system packages.
More information there : https://virtualenv.pypa.io/en/latest/userguide/#the-system-site-packages-option


HowTo install
^^^^^^^^^^^^^

To install it::

  pip install pyros_setup

You need to be careful which environmnt you install this package in:

- If you are running in a virtual environment `myvenv`, and you want to access ROS packages outside this environment (since they should be installed on the system in `/opt/ros/$ROS_DISTRO`), you need to install pyros-setup in the virtual environment `myvenv`. Please report bugs you may encounter, this is the proper way to setup the environment without risks to break your system python, but likely not the most tested...
- If you are running the system python interpreter (not in a virtual environment), you should then install this package on your system (in `/usr/local/lib/python2.7/site-packages`, also via the same pip command). Although this is definitely not a recommended way to setup python packages, it is the defacto standard way for ROS.

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


HowTo develop
^^^^^^^^^^^^^

After cloning the repository, you can use pipenv to setup your development environment.
Be careful to allow access to system packages::

  pipenv --two --system-packages

HowTo use
^^^^^^^^^

Basically pyros_setup allows you to do this::

  try:
      import rospy
      import roslaunch
      import rosgraph
      import rosnode
  except ImportError:  # if ROS environment is not setup, we emulate it.
      import pyros_setup
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

If you want to use pyros-setup as a dependency of your package, you should depend on the pip package.
Afterall, the whole point is to provide access to ROS from a pure python environment.


Python3
^^^^^^^

DISCLAIMER: This is not compatible with catkin_pip usage. Also NOT SUPPORTED.
Attempt this at your own risk.

But since python2 is almost dead, we might want to move to python3 already.
Just remember that ROS does not support python3 out of the box, and significant tinkering is required.

For python3, you will need to install dependent python packages in your python3 virtual environment.
For example, pyros-setup itself only requires rospkg and pyyaml for the tests to pass::

  pip3 install pyyaml rospkg


More information there : https://answers.ros.org/question/237613/how-to-define-ros-kinetic-to-use-python3-instead-of-python27/


Remarks
^^^^^^^

Although it would technically be possible to build a ROS package from this source, this will NOT be done.
The catkin_pip build system that was here once, was only here to help having pyros-setup in a source workspace while developing on it.
When using ROS directly this package is not needed, and having it installed among ROS packages would cause much useless confusion with python import mechanism.


Roadmap
^^^^^^^

- The way forward seems to build a ROS wheel from source, with the basic packages inside...
This will make it compatible with any python environment, easily installable, and isolate it from the operating system.

- This code might eventually be migrated into rosimport, which, as a larger scope, focuses on python environments interoparability with ROS.

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
