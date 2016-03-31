cmake_minimum_required(VERSION 2.8.3)
project(pyros_setup)

include(cmake/python-venv.cmake)

# For ease of use, this CMakeLists allows to use this code from source, as if it were a normal catkin package.
# However no ROS/catkin debian package is provided for it.
# It is just a pip package that your packages should depend on (using rosdep dependency mechanism).
# We should follow closely https://github.com/gerkey/ros1_external_use

# Using our own catkin extras
include(cmake/catkin-pip.cmake)

# Minimal Python module setup - catkin is assumed preinstalled along with ROS
# This assume catkin is installed along with a minimal ROS system.
find_package(catkin REQUIRED COMPONENTS
    rospy
    rosgraph
    roslaunch
    rostest
)

# Building a virtualenv in catkin workspace

# installing requirements

catkin_package()
catkin_python_venv()

# We need to do the catkin python setup to create the workspace
# before installing pip requirements
# catkin_python_setup() # MAYBE NOT

# We need to install the pip dependencies in the workspace being created
catkin_pip_install_requirements()

# Installing cmake extras in case others want to use it (maybe useless?)
#set(${PROJECT_NAME}_MODULES "")
#list(APPEND ${PROJECT_NAME}_MODULES catkin-pip.cmake)

#catkin_package(
#  CFG_EXTRAS ${${PROJECT_NAME}_MODULES}
#)

#install(DIRECTORY cmake
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)


## Unit tests
if (CATKIN_ENABLE_TESTING)

    ############
    # Nose Tests
    ############

    catkin_add_nosetests(pyros_setup/tests)
    # this test needs to be executable for rostest, so we need to add it specifically for nose
    catkin_add_nosetests(pyros_setup/tests/test_rostest_nose.py)

    ###########
    # ROS Tests
    ###########

    find_package(rostest REQUIRED)
    add_rostest(pyros_setup/tests/test_rostest_nose.test)

endif()
