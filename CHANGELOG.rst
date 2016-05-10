^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pyros_setup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2016-05-10)
------------------
* adding missing config package to setup.py
* ros package still depending on catkin afterall
* modifying ros utils script to improve debug for python and pip behavior
* helping debug of rosutils scripts
* rosutils scripts changing to build directory before doing anything else
* now using package v2 format
* fixing rosutils scripts.
* adding package version
* mention shadowrobot buildtools in readme.
* attempting matrix build.
* fixing virtualenvwrapper setup script path on ubuntu.
  removed debian_frontend already setup on travis trusty image.
* fixing virtualenvwrapper setup.
* force yes for python virtualenv install.
* adding shell script to isolate ros setup during travis test.
  improved travis build to test usage from both python venv and ROS.
* now running python test (in venvs) from travis.
* set next version number. cosmetics.
* added a default config file to be used by client programs for default ros configuration.
  now using importlib instead of custom import_string()
  separated packagebound, confighandler, and config import classes
  simplified setup
  fixed tests
* master is now default branch. fixing travis badge url.
* fiddling around with configuration to make it usable from pyros.
* first version after refactor to handle config file. good enough for self tests to use it.
* fixes for latest catkin_pure_python.
  readme improvements
* improving readme
* cleanup doc and comments.
* now depending on catkin_pure_python.
* working pip install requirements in catkin workspace
* added simple method to get ros_home.
* first verison of cmake creating a venv to store packages.
* added comments...
* fixing pip install command
* trying to install pip requirements ni devel space. notworking yet
* comments
* broken cmake stub for catkin-pip
* setting cmake as buildtool
* improved error message when ROs setup fails
* first experiment with using a virtualenv in devel workspace.
* fixed logic for ros_package_path when not a devel workspace.
  cosmetics.
* Contributors: AlexV, alexv

0.0.12 (2016-02-10)
-------------------
* v0.012
* fixing nose dependency version and removing test pip dependency on catkin package pyros_test
* Contributors: AlexV, alexv

0.0.11 (2016-01-26)
-------------------
* preparing 0.0.11
* adding simple entrypoint to run nose tests
* now running setup.py only with setuptools.
  adding nose.collector as test runner for setup.py.
  shutting down roslaunch before exiting tests.
* Contributors: AlexV

0.0.10 (2016-01-25)
-------------------
* Revert "removing executable flag from test_rostest_nose since it can now be executed with nose as well as rostest"
  This reverts commit b915beba3731eb03c1bd187bba05af1c337e8034.
* removing executable flag from test_rostest_nose since it can now be executed with nose as well as rostest
* improving travis tests to run on devel and install version.
* replacing talker test node by a pyros_test node to not change dependency list.
  also make rostest wait on it, otherwise it can fail.
* preparing v0.0.10
* improved tests for rostest_nose module to make sure rostest still works.
* Contributors: AlexV, alexv

0.0.9 (2016-01-09)
------------------
* prepring 0.0.9
* adding nose in requirements.txt
  removing unprotected catkin import in setup.py
* Contributors: AlexV

0.0.8 (2016-01-08 20:16)
------------------------
* preparing 0.0.8
* adding catkin_package() cmake command
* Contributors: alexv

0.0.7 (2016-01-08 18:07)
------------------------
* preparing 0.0.7
* cleaning up dependencies since uneeded python-six breaks buildfarm for EOLed saucy
* Contributors: alexv

0.0.6 (2016-01-08 15:48)
------------------------
* preparing 0.0.6
* adding parameter to get_master in the case delayed_import is not called
* improved dynamic module behavior.
* improving module for delayed import
* improved README rst formatting
* added code samples to README to make aim clear
* change doc in README to explicitely target python package
* Contributors: alexv

0.0.5 (2016-01-08 11:55)
------------------------
* version to 0.0.5
* readding package.xml in egg while we use catkin_pkg to break the egg
* Contributors: alexv

0.0.4 (2016-01-07 20:40)
------------------------
* preparing for 0.0.4 pypi release
* adding useful files for pypi release.
* playing with python sdist and eggs for release on pypi.
* adding gitignore to hide those .pyc
* using shadow-fixed repo for travis
  This way we can get latest dependency to test latest version of source, which probably makes more sense than testing stable.
* Contributors: AlexV, alexv

0.0.3 (2016-01-07 18:30:07 +0900)
---------------------------------

0.0.2 (2016-01-07 18:30:07 +0900)
---------------------------------
* changing package version to 0.0.2
* todo comment for detecting default distro
* added delayed_import_auto to make workspace discovery explicit.
  simplified implementation (most methods deal with only one workspace at a time)
  improved tests.
* fixing tests
* better workaround for ros_comm issue 711
* adding check to teardown module, to make sure roscore is really dead.
* fixing tests, no matter the time it takes to start/stop processes.
* adding pyros_test as test dependency.
* adding finally clause to test to cleanup even if tests fail.
* cosmetics
* adding travis badge.
* Contributors: alexv

0.0.1 (2016-01-04)
------------------
* adding rosnode as testdependency. cosmetics.
* fixing tests shutting down properly.
* fix direct import when ROS is already setup
  now returning roscore_process when getting master to allow termination.
  tests still broken
* adding nosetests command to travis file
* adding travis file
* renamed ROS_Master to get_master since we return the same as the rospy function.
* improved delayed import to work recursively if needed
* fixed ordered dict to keep env vars ordering and remove checks that might break this ordering.
* improved __init_\_ to delay setup and imports. now testing node starting.
  moved testpkg in separate repo.
* adding test for core and launch. added base structure for test pkg.
* addded rospy import test
* first commit, extracted code from pyros
* Contributors: AlexV, alexv
