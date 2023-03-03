#!/bin/bash
NAO_IP1=192.168.10.219
SHM_ID1=img0
RES1=1 #1 for QVGA, 2 for VGA
CAM1=0 #0 for bottom, 1 for top

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

EXE1=`echo $SCRIPT_DIR`/src/rosnao_wrapper/build/rosnao_wrapper_imagepub
chmod +x `echo $EXE1`
sh `echo $EXE1` `echo $NAO_IP1` `echo $SHM_ID1` `echo $RES1` `echo $CAM1`

# (trap 'kill 0' SIGINT; prog1 & prog2 & prog3)