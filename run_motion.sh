
NAO_IP=192.168.10.24
SHM_ID=motion

SCRIPT_DIR=$(dirname `echo $(realpath "$0")`)
source `echo $SCRIPT_DIR`/devel/setup.bash

roslaunch rosnao_bridge motion_relay.launch nao_ip:=`echo $NAO_IP` shm_id:=`echo $SHM_ID`