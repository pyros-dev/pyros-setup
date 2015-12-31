cmake_minimum_required(VERSION 2.8.3)
project(pyros_setup)

# Minimal Python module setup
find_package(catkin REQUIRED COMPONENTS
    roslint
    rospy
)
catkin_python_setup()
# Lint Python modules
file(GLOB_RECURSE ${PROJECT_NAME}_PY_SRCS
     RELATIVE ${PROJECT_SOURCE_DIR} pyros_setup/*.py)
roslint_python(${${PROJECT_NAME}_PY_SRCS})

## Unit tests
if (CATKIN_ENABLE_TESTING)

    ##############################################################################
    # Nose Tests
    ##############################################################################

    catkin_add_nosetests(pyros_setup/tests)


endif()
