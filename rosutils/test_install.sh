#!/usr/bin/env bash
cd build
source install/setup.bash

echo $PYTHONPATH
py.test --pyargs pyros_setup
