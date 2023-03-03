#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
(cd `echo $SCRIPT_DIR`; chmod +x clean_make_sdk.sh; sh clean_make_sdk.sh)
echo "=========================== SDK MAKE DONE =============================="
(cd `echo $SCRIPT_DIR`; chmod +x clean_make.sh; sh clean_make.sh)