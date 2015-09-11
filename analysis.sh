#!/bin/bash
# Compile QuadTarget (if needed) and run it in image analysis mode -- spits
# data about detected markers to stdout, is filtered a bit and then saved to
# whatever file you specify as a commandline argument.

if [ "$1" = "" ]; then
    echo "Please give /path/to/desired_output.csv"
    echo "Usage: $0 /path/to/output.csv markers|time"
    exit 1
fi

if [ "$2" = "" ]; then
    echo "Please select one of [markers, time]"
    echo "Usage: $0 /path/to/output.csv markers|time"
    exit 1
fi

QTFSM_DIR="/home/owain/Dropbox/Aber/MSc_Dissertation/code/QuadTargetFSM"
export QUADTARGET_CONFIGS="$QTFSM_DIR/cfg/defaults.ini"
case "$2" in
    markers)
        CONFIGS="$QUADTARGET_CONFIGS $QTFSM_DIR/cfg/config_images.ini"
        SED='s/^null//g'
        ;;
    time)
        CONFIGS="$QUADTARGET_CONFIGS $QTFSM_DIR/cfg/config_video.ini $QTFSM_DIR/cfg/config_profiling.ini"
        SED='s/\w+=//g'
        ;;
    *)
        echo "Please select one of [markers, time]"
        echo "Usage: $0 /path/to/output.csv markers|time"
        exit 1
esac
mkdir -p $HOME/Projects/quadtargetfsm
cd $HOME/Projects/quadtargetfsm
cmake -DCMAKE_BUILD_TYPE=Release $QTFSM_DIR
ccache make -j8
export QUADTARGET_CONFIGS="$QUADTARGET_CONFIGS $CONFIGS"
./QuadTarget | sed -E $SED | uniq | tee $1
