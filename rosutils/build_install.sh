#!/usr/bin/env bash
cd build

echo $PYTHONPATH
make -j1 install
