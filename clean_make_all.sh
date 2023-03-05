#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
(cd `echo $SCRIPT_DIR`; sh clean_make_sdk.sh)
echo "=========================== SDK MAKE DONE =============================="
(cd `echo $SCRIPT_DIR`; sh clean_make.sh)