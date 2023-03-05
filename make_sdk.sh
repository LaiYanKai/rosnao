#!/bin/bash

SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
(cd `echo $SCRIPT_DIR`/src/rosnao_wrapper; qibuild make)
