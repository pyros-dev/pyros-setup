cmake_minimum_required(VERSION 2.8.3)
project(pyros_setup)

# For ease of use, this CMakeLists allows to use this code from source, as if it were a normal catkin package.
# However no ROS/catkin debian package is provided for it.
# It is just a pip package that your packages should depend on (using rosdep dependency mechanism).

# Using our own catkin extras
include(cmake/catkin-pip.cmake)

# Minimal Python module setup - catkin is assumed preinstalled along with ROS
find_package(catkin REQUIRED COMPONENTS
    rospy
)

# We need to install the pip dependencies in the workspace being built
catkin_pip_install()

# Installing cmake extras
set(${PROJECT_NAME}_MODULES "")
list(APPEND ${PROJECT_NAME}_MODULES catkin-pip.cmake)

catkin_package(
  CFG_EXTRAS ${${PROJECT_NAME}_MODULES}
)

install(DIRECTORY cmake
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


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
