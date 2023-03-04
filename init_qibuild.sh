#!/bin/bash
SDK_DIR=~/naoqi_sdk_cpp

if [[ ! -f `echo $SDK_DIR`/toolchain.xml ]] ; then
    echo 'SDK not found'
    exit
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)

# adapted from http://doc.aldebaran.com/2-8/dev/cpp/install_guide.html
pip install qibuild --user
echo "PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc # adds the line in "" into .bashrc

# manual input here
echo "---------------"
echo "You will be prompted to specify:"
echo "1. CMake path (if not automatically found. Should be found when build-essential is installed),"
echo "2. CMake Generator (Choose 'Unix Makefiles')"
echo "3. the IDE you will use. (Choose 'None')"
echo "---------------"
qibuild config --wizard

cd `echo $SCRIPT_DIR`/src/rosnao_wrapper
# qibuild init # should already been initialised (in git code)

qitoolchain create mtc `echo $SDK_DIR`/toolchain.xml
qibuild add-config mcfg -t mtc --default