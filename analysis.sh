#!/bin/bash
# Compile QuadTarget (if needed) and run it in image analysis mode -- spits
# data about detected markers to stdout, is filtered a bit and then saved to
# whatever file you specify as a commandline argument.

if [ "$1" = "" ]; then
    echo "Please give /path/to/desired_output.csv
    exit 1
fi

QTFSM_DIR="/home/owain/Dropbox/Aber/MSc_Dissertation/code/QuadTargetFSM"
export QUADTARGET_CONFIGS="$QTFSM_DIR/cfg/defaults.ini $QTFSM_DIR/cfg/config_images.ini"
mkdir -p $HOME/Projects/quadtargetfsm
cd $HOME/Projects/quadtargetfsm
cmake -DCMAKE_BUILD_TYPE=Release $QTFSM_DIR
ccache make -j8
./QuadTarget | sed 's/^null//g' | uniq | tee $1
