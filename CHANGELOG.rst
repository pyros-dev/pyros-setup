Changelog
=========

%%version%% (unreleased)
------------------------

- Commenting travis check breaking because of pytest bug with installed
  tests. [alexv]

- Bumping pyros-config requirement. fixed release flow description.
  [alexv]

- Tagging changelog. [alexv]

0.1.4 (2016-08-31)
------------------

- V0.1.4. [alexv]

- Updating changelog and setup.py release flow comment. [alexv]

- Improved travis test to catch config file errors. [alexv]

- Refined release flow in setup.py comments. [alexv]

- Fixed quoting distro in generated config file. [alexv]

0.1.3 (2016-08-31)
------------------

- V0.1.3. [alexv]

- Generating changelog. [alexv]

- Attempting to get travis to workaround https://github.com/pytest-
  dev/pytest/issues/1889. [alexv]

- Now detecting ros distro on config generation. increased pyros_config
  requirement to support config generation. [alexv]

- Reviewed test and logging configuration. [alexv]

- Now using pyros_setup command to run self test instead of pytest.
  simplified travis check, dropping ros workflow. [alexv]

- Attempting default config file generation. added troubleshooting
  section to README. [alexv]

- Remove __init__.py from test folder as detailed in pytest doc. bumped
  version to 0.1.99 to denote devel. [alexv]

- Improved setup.py with custom commands. Reviewed test, now using
  py.test -s. Fixed some details in README. [alexv]

- Adding method to generate config file in instance folder. [alexv]

- Adding instructions for running tox. [alexv]

- Following pip format to display version. [alexv]

- Refining config and tests, to fail tests if default config is wrong
  for user setup. [alexv]

- Adding kinetic configs. [alexv]

- Comments for release workflow. [alexv]

- Fix typo in log message. [alexv]

0.1.2 (2016-08-11)
------------------

- V0.1.2. [alexv]

- Changelog. [alexv]

- Turning pyros-setup in a pure python package, since it should never be
  used with ROS environment setup anyway. [alexv]

- Added __file__ and __version__ to pyros_setup import relay class.
  improved deprecated decorator. improved warnings. [alexv]

- Improving main entrypoint with --version and --help. [alexv]

- Improved main entry point for self test. [alexv]

- Added site-packages to PYTHONPATH to support catkin_pip hybrid
  workspaces. replaced deprecated logging methods. [alexv]

0.1.1 (2016-08-11)
------------------

- Preparing 0.1.1. [alexv]

- Quick docs cleanup. [alexv]

- Cosmetics. [alexv]

- Requiring catkin_pip minimum 0.1.10, in order to fail on ros install
  flow since some of our pip dependencies are not satisfied by ros debs
  (and we dont want them to in this python package). [alexv]

- Disabling travis checks for ros install flow since this package is not
  aimed at being installed or released as a ROS package. [alexv]

- Review documentation. fixing travis test. [alexv]

- Renaming DISTRO -> CI_ROS_DISTRO variable from travis config. [alexv]

- Separating devel and install flow for ros travis checks. added publish
  and tag methods to setup.py an removed pypi_release script. improved
  setup.cfg and added doc requirements. [alexv]

- Reviewing travis for test and cosmetics. [AlexV]

- Reviewing tox and tests. [AlexV]

- Comments for tox. [AlexV]

- New test strategy for travis. since we need to use system packages, we
  cant use travis python language build environment. [AlexV]

- Forcing travis to get latest tox. [AlexV]

- Fixed tox.ini to set venvs properly and make tests pass. [AlexV]

- Fixing tests to be run with pytest from catkin. [AlexV]

- Fixed main entrypoint for pytest. fixed pyros-utils and pyros-config
  dependencies version. [AlexV]

- Now adding path to ROS_PACKLAGE_PATH only if it exists. [alexv]

- Fixing test for pytest. [alexv]

- Travis now using py.test directly since tox doesnt work refined
  tox.ini improving gitignore. [alexv]

- Now using pytest for self test. [alexv]

- Using renamed catkin_pip. [alexv]

- Setting up python environments and tox. fixing version in setup.py.
  [alexv]

- Restructuring to a pure python package with pytest and tox testing, on
  indigo and jade. pytest passing (remove *.pyc). tox failing. [alexv]

- Cleaning up pyros_utils related code. [alexv]

- Cleaning up pyros_config related code. [alexv]

- Now requiring catkin_pure_python 0.1.2. comments. [alexv]

- Updating for catkin_pure_python 0.1.0. [AlexV]

- Bumping minimum required catkin_pure_python version to 0.0.8 to not
  write to install workspace before make install. [alexv]

- Improved readme, mentioning to use pip package and catkin_pure_python.
  [alexv]

- Hopefully fixing travis build dependencies problem, for now... [alexv]

- Improved error message when import fails. [alexv]

- Fixing rosdep install step in travis. [alexv]

- Now installing ros dependencies in ros build script to check ros
  dependencies can also be retrieved from python workflow. [alexv]

- Added doc about config file contents. [alexv]

- Updated docs. cosmetics. [alexv]

0.1.0 (2016-05-10)
------------------

- Generating changelog. [alexv]

- Adding missing config package to setup.py. [alexv]

- Ros package still depending on catkin afterall. [alexv]

- Modifying ros utils script to improve debug for python and pip
  behavior. [AlexV]

- Helping debug of rosutils scripts. [alexv]

- Rosutils scripts changing to build directory before doing anything
  else. [AlexV]

- Now using package v2 format. [alexv]

- Fixed logic for ros_package_path when not a devel workspace.
  cosmetics. [alexv]

- Master is now default branch. fixing travis badge url. [alexv]

- Fixing rosutils scripts. [alexv]

- Adding package version. [alexv]

- Mention shadowrobot buildtools in readme. [alexv]

- Attempting matrix build. [alexv]

- Fixing virtualenvwrapper setup script path on ubuntu. removed
  debian_frontend already setup on travis trusty image. [alexv]

- Fixing virtualenvwrapper setup. [alexv]

- Force yes for python virtualenv install. [alexv]

- Adding shell script to isolate ros setup during travis test. improved
  travis build to test usage from both python venv and ROS. [alexv]

- Now running python test (in venvs) from travis. [alexv]

- Set next version number. cosmetics. [alexv]

- Added a default config file to be used by client programs for default
  ros configuration. now using importlib instead of custom
  import_string() separated packagebound, confighandler, and config
  import classes simplified setup fixed tests. [alexv]

- Fiddling around with configuration to make it usable from pyros.
  [alexv]

- First version after refactor to handle config file. good enough for
  self tests to use it. [alexv]

- Fixes for latest catkin_pure_python. readme improvements. [alexv]

- Improving readme. [alexv]

- Cleanup doc and comments. [alexv]

- Now depending on catkin_pure_python. [alexv]

- Working pip install requirements in catkin workspace. [alexv]

- Added simple method to get ros_home. [alexv]

- First experiment with using a virtualenv in devel workspace. [AlexV]

- First verison of cmake creating a venv to store packages. [AlexV]

- Added comments... [alexv]

- Fixing pip install command. [alexv]

- Trying to install pip requirements ni devel space. notworking yet.
  [AlexV]

- Comments. [AlexV]

- Broken cmake stub for catkin-pip. [AlexV]

- Setting cmake as buildtool. [AlexV]

- Improved error message when ROs setup fails. [AlexV]

0.0.12 (2016-02-10)
-------------------

- V0.012. [AlexV]

- Fixing nose dependency version and removing test pip dependency on
  catkin package pyros_test. [alexv]

0.0.11 (2016-01-26)
-------------------

- Preparing 0.0.11. [AlexV]

- Adding simple entrypoint to run nose tests. [AlexV]

- Now running setup.py only with setuptools. adding nose.collector as
  test runner for setup.py. shutting down roslaunch before exiting
  tests. [AlexV]

0.0.10 (2016-01-25)
-------------------

- Revert "removing executable flag from test_rostest_nose since it can
  now be executed with nose as well as rostest" [alexv]

  This reverts commit b915beba3731eb03c1bd187bba05af1c337e8034.

- Removing executable flag from test_rostest_nose since it can now be
  executed with nose as well as rostest. [alexv]

- Improving travis tests to run on devel and install version. [alexv]

- Replacing talker test node by a pyros_test node to not change
  dependency list. also make rostest wait on it, otherwise it can fail.
  [alexv]

- Preparing v0.0.10. [alexv]

- Improved tests for rostest_nose module to make sure rostest still
  works. [alexv]

0.0.9 (2016-01-09)
------------------

- Prepring 0.0.9. [AlexV]

- Adding nose in requirements.txt removing unprotected catkin import in
  setup.py. [AlexV]

0.0.8 (2016-01-08)
------------------

- Preparing 0.0.8. [alexv]

- Adding catkin_package() cmake command. [alexv]

0.0.7 (2016-01-08)
------------------

- Preparing 0.0.7. [alexv]

- Cleaning up dependencies since uneeded python-six breaks buildfarm for
  EOLed saucy. [alexv]

0.0.6 (2016-01-08)
------------------

- Preparing 0.0.6. [alexv]

- Adding parameter to get_master in the case delayed_import is not
  called. [alexv]

- Improved dynamic module behavior. [alexv]

- Improving module for delayed import. [alexv]

- Improved README rst formatting. [alexv]

- Added code samples to README to make aim clear. [alexv]

- Change doc in README to explicitely target python package. [alexv]

0.0.5 (2016-01-08)
------------------

- Version to 0.0.5. [alexv]

- Readding package.xml in egg while we use catkin_pkg to break the egg.
  [alexv]

0.0.4 (2016-01-07)
------------------

- Preparing for 0.0.4 pypi release. [alexv]

- Using shadow-fixed repo for travis. [AlexV]

  This way we can get latest dependency to test latest version of source, which probably makes more sense than testing stable.

- Adding gitignore to hide those .pyc. [alexv]

- Adding useful files for pypi release. [alexv]

- Playing with python sdist and eggs for release on pypi. [alexv]

0.0.2 (2016-01-07)
------------------

- Changing package version to 0.0.2. [alexv]

- Todo comment for detecting default distro. [alexv]

- Added delayed_import_auto to make workspace discovery explicit.
  simplified implementation (most methods deal with only one workspace
  at a time) improved tests. [alexv]

- Fixing tests. [alexv]

- Better workaround for ros_comm issue 711. [alexv]

- Adding check to teardown module, to make sure roscore is really dead.
  [alexv]

- Fixing tests, no matter the time it takes to start/stop processes.
  [alexv]

- Adding pyros_test as test dependency. [alexv]

- Adding finally clause to test to cleanup even if tests fail. [alexv]

- Cosmetics. [alexv]

- Adding travis badge. [alexv]

0.0.1 (2016-01-04)
------------------

- Adding rosnode as testdependency. cosmetics. [alexv]

- Fixing tests shutting down properly. [alexv]

- Fix direct import when ROS is already setup now returning
  roscore_process when getting master to allow termination. tests still
  broken. [alexv]

- Adding nosetests command to travis file. [AlexV]

- Adding travis file. [AlexV]

- Renamed ROS_Master to get_master since we return the same as the rospy
  function. [alexv]

- Improved delayed import to work recursively if needed. [alexv]

- Fixed ordered dict to keep env vars ordering and remove checks that
  might break this ordering. [alexv]

- Improved __init__ to delay setup and imports. now testing node
  starting. moved testpkg in separate repo. [alexv]

- Adding test for core and launch. added base structure for test pkg.
  [alexv]

- Addded rospy import test. [alexv]

- First commit, extracted code from pyros. [alexv]


