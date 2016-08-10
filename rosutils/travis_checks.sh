#!/usr/bin/env bash
set -e

# These variables need to be setup before calling this script:
# CI_ROS_DISTRO [indigo | jade]
# ROS_FLOW [devel | install]

if [ "$ROS_FLOW" == "devel" ]; then
    source devel/setup.bash

    echo PYTHONPATH = $PYTHONPATH
    make -j1 tests
    make -j1 run_tests
    catkin_test_results .
elif [ "$ROS_FLOW" == "install" ]; then
    source install/setup.bash

    echo PYTHONPATH = $PYTHONPATH
    python -m pytest --pyargs pyzmp
fi
