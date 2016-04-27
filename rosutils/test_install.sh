#!/usr/bin/env bash
set -x
cd build
source install/setup.bash
nosetests pyros_setup.tests
