#!/bin/bash
SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
(cd `echo $SCRIPT_DIR`; sh clean_make_sdk.sh)
echo "=========================== SDK MAKE DONE =============================="
(cd `echo $SCRIPT_DIR`; sh clean_make.sh)