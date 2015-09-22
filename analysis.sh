#!/bin/bash
# Compile QuadTarget (if needed) and run it in image analysis mode -- spits
# data about detected markers to stdout, is filtered a bit and then saved to
# whatever file you specify as a commandline argument.

if [ "$1" = "" ]; then
    echo "Please give /path/to/desired_output.csv"
    echo "Usage: $0 /path/to/output.csv markers|time|time_pi"
    exit 1
fi

if [ "$2" = "" ]; then
    echo "Please select one of [markers, time]"
    echo "Usage: $0 /path/to/output.csv markers|time|pi"
    exit 1
fi

CUSTOM_VIDEO=""
if [ "$3" != "" ]; then
    CUSTOM_VIDEO=$3
fi

EXTRA_CONFIG=""
if [ "$4" != "" ]; then
    EXTRA_CONFIG=$4
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
    time_pi)
        QTFSM_DIR="/root/quadtargetfsm"
        export QUADTARGET_CONFIGS="$QTFSM_DIR/cfg/defaults.ini $QTFSM_DIR/cfg/pi/defaults.ini"
        CONFIGS="$QTFSM_DIR/cfg/pi/config_video.ini $QTFSM_DIR/cfg/config_profiling.ini"
        SED='s/\w+=//g'
        ;;
    *)
        echo "Please select one of [markers, time, time_pi]"
        echo "Usage: $0 /path/to/output.csv markers|time|time_pi"
        exit 1
esac
if [ "$2" = "time_pi" ]; then
    cd /root/quadtargetfsm
    ./pi_build.sh
    cd build
else
    mkdir -p $HOME/Projects/quadtargetfsm
    cd $HOME/Projects/quadtargetfsm
    cmake -DCMAKE_BUILD_TYPE=Release $QTFSM_DIR
    ccache make -j
fi
export QUADTARGET_CONFIGS="$QUADTARGET_CONFIGS $CONFIGS $EXTRA_CONFIG"
nice -n -10 ./QuadTarget none $CUSTOM_VIDEO | sed -E $SED | uniq > $1
