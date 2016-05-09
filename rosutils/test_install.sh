#!/usr/bin/env bash
cd build
source install/setup.bash

echo $PYTHONPATH
nosetests pyros_setup.tests
