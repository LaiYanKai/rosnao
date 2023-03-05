#!/bin/bash

SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
BUILD_DIR=`echo $SCRIPT_DIR`/src/rosnao_wrapper/build-mcfg
(rm -rf `echo $BUILD_DIR`/*; cd `echo $BUILD_DIR`/..; qibuild configure ) # avoid * in rm -rf in case dir doesn't exist
sh make_sdk.sh

#BUILD_DIR=`echo $SCRIPT_DIR`/rosnao_wrapper/build
# mkdir -p `echo $BUILD_DIR`
# rm -rf `echo $BUILD_DIR`/*
# cd `echo $BUILD_DIR`
# cmake ..
# cmake --build .