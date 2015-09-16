#!/usr/bin/env bash
# The Pi Camera v4l2 driver needs to be asked to set the resolution and target
# framerate before we use it with OpenCV.
WIDTH=1296
HEIGHT=730
FPS=49
FLIP_HORIZ=0
FLIP_VERT=1

v4l2-ctl -p $FPS
v4l2-ctl --set-fmt-video=width=$WIDTH,height=$HEIGHT,pixelformat=2
v4l2-ctl --set-ctrl=horizontal_flip=$FLIP_HORIZ,vertical_flip=$FLIP_VERT
export QTFSMCFG = "/root/quadtargetfsm/cfg"
export QUADTARGET_CONFIGS="$QTFSMCFG/defaults.ini $QTFSMCFG/pi/defaults.ini"
cd /root/quadtargetfsm/build
./QuadTarget
