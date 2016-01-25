cmake_minimum_required(VERSION 2.8.3)
project(pyros_setup)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS
    roslint
    rospy
)
catkin_python_setup()
catkin_package()

# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} pyros_setup/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})

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
