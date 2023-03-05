#!/bin/bash

# Launches the Video Device proxy in Nao, and puts the image into shared memory
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
BIN_DIR=`echo $SCRIPT_DIR`/../../rosnao_wrapper/build-mcfg/sdk/bin

(cd `echo $BIN_DIR`; chmod +x `echo imagepub`; ./image_pub $1 $2 $3 $4) #IP , SHM_ID, RES, CAM
