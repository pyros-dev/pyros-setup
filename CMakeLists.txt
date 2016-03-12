cmake_minimum_required(VERSION 2.8.3)
project(pyros_setup)

include(cmake/python-venv.cmake)

# Minimal Python module setup
# This assume catkin is installed along with a minimal ROS system.
find_package(catkin REQUIRED COMPONENTS
    rospy
    rosgraph
    roslaunch
    rostest
)

# Building a virtualenv in catkin workspace

# installing requirements

#catkin_python_setup()
catkin_package()
catkin_python_venv()

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
