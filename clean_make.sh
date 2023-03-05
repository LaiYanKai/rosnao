#!/bin/bash
SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
(cd `echo $SCRIPT_DIR`; rm -rf build devel src/CMakeLists.txt; catkin_make)