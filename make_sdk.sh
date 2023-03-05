#!/bin/bash

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
(cd `echo $SCRIPT_DIR`/src/rosnao_wrapper; qibuild make)
