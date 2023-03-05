
NAO_IP=192.168.10.24
SHM_ID=img

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
source `echo $SCRIPT_DIR`/devel/setup.bash

chmod +x `echo $SCRIPT_DIR`/src/rosnao_bridge/scripts/motion_relay.sh
roslaunch rosnao_bridge motion_relay.launch nao_ip:=`echo $NAO_IP` shm_id:=`echo $SHM_ID`