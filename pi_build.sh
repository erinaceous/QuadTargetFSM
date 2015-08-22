#!/bin/bash

cd /root/quadtargetfsm
git pull
mkdir -p build
cd build
EXTRA_BUILD_FLAGS="-mtune=cortex-a7 -mfpu=neon-vfpv4 -funsafe-math-optimizations"
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_VFPV3=ON -DENABLE_VFPV4=ON -DENABLE_NEON=ON -DWITH_TBB=ON -DBUILD_TBB=ON -DCMAKE_CXX_FLAGS=$EXTRA_BUILD_FLAGS ../
ccache make -j3
cp ../cfg/config_pi.ini config.ini
<<<<<<< HEAD
cp ../
=======
cp ../cfg/config_pi* ./
>>>>>>> 1a297b30c0e7d1ab87a34a719ce681cd936dcf5c
