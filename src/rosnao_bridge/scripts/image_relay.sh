#!/bin/bash
SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
source `echo $SCRIPT_DIR`/get_bin_dir.sh

# Launches the Video Device proxy in Nao, and puts the image into shared memory
(cd `echo $BIN_DIR`; chmod +x image_pub; ./image_pub $1 $2 $3 $4) #IP, SHM_ID, RES, CAM
exit 0