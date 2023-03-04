#!/bin/bash

SHM_ID1=img0
RES1=1 #1 for QVGA, 2 for VGA
TOPIC1="cam0"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
source `echo $SCRIPT_DIR`/devel/setup.bash
roslaunch rosnao_bridge image_relay.launch args1:="`echo $SHM_ID1` `echo $RES1` `echo $TOPIC1`"
