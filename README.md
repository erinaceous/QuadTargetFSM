# QuadTargetFSM

![What a landing target looks like](https://raw.githubusercontent.com/erinaceous/QuadTargetFSM/master/Target.png)

Owain Jones <contact@odj.me>, MSc Dissertation project

C++ and OpenCV implementation of quadcopter target tracking algorithm.
This is my cleaned up version that uses a finite state machine to detect the
marker patterns. Working slightly better (AKA can actually track targets)
and more efficiently and less buggily than the original implementation.

It has a number of variables that can be tweaked -- see `cfg/defaults.ini` for
detailed comments on all of the algorithms' parameters. By default it is tuned
for detecting targets in outdoors lighting using a Raspberry Pi NoIR camera.

Building
========

It's pretty light on dependencies, but needs
* CMake version 3.0 at minimum
* Modern UNIX environment (may or may not compile on Mac. YMMV)
* OpenCV (libopencv-dev / opencv / opencv-devel)
* Boost (libboost-dev / boost / libboost-devel)
* Doxygen (optional) for generating documentation.

Assuming you have those installed with whatever package manager your distro
uses, building is simple:

    git clone https://github.com/erinaceous/QuadTargetFSM.git
    mkdir QuadTargetFSM/build
    cd QuadTargetFSM/build
    cmake -DCMAKE_BUILD_TYPE=Release ../
    make all
    
Building on a Raspberry Pi
==========================

See `pi_build.sh` for a few optimizations for the Raspberry Pi 2; enabling the
use of the NEON floating point unit and so on.

Running
=======

Before you can run QuadTarget, you need to specify what configuration files
it should read via the `QUADTARGET_CONFIGS` environment variable. This is a
string of paths to `ini` files, separated by spaces, for example:

    export QUADTARGET_CONFIGS="../cfg/defaults.ini ../cfg/config_cascade.ini"
    
`defaults.ini` has sensible values for all the config variables, and every
configuration file loaded overrides the values specified in the previous config
file loaded. By default QuadTarget will attempt to run using the first webcam
on your system.

To actually run it, assuming you've set `QUADTARGET_CONFIGS`:

    ./QuadTarget [optional /path/to/output_video.avi] [optional /path/to/input_video.h264]
    
It will look in the config files for input video / output video paths first, but
these will be overridden if the optional command line arguments are given.
