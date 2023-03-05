
NAO_IP=192.168.10.24
SHM_ID=img
RES=1 #1 for QVGA, 2 for VGA
CAM=0 #0 for bottom, 1 for top
TOPIC="cam"
FRAME="world"

SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
source `echo $SCRIPT_DIR`/devel/setup.bash

chmod +x `echo $SCRIPT_DIR`/src/rosnao_bridge/scripts/*.sh
roslaunch rosnao_bridge image_relay.launch nao_ip:=`echo $NAO_IP` shm_id:=`echo $SHM_ID` res:=`echo $RES` cam:=`echo $CAM` topic:=`echo $TOPIC` frame:=`echo $FRAME`