#!/bin/bash

SHM_ID1=img0
RES1=1 #1 for QVGA, 2 for VGA

source devel/setup.bash
roscore & rosrun rosnao_bridge rosnao_bridge_test `echo $SHM_ID1` `echo $RES1`

# (trap 'kill 0' SIGINT; prog1 & prog2 & prog3)