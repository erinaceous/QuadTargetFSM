#!/bin/bash
# Compile for Raspberry Pi 2! Tries to force GCC to compile with optimizations
# for the ARM8 processor of the Pi2 -- enabling the VFP and NEON floating point
# units.

cd /root/quadtargetfsm
git pull
wget -o picam/picam.zip http://www.cheerfulprogrammer.com/downloads/picamtutorial/picamdemo.zip
unzip picam/picam.zip -d picam/
mkdir -p build
cd build
EXTRA_BUILD_FLAGS="-mtune=cortex-a7 -mfpu=neon-vfpv4 -funsafe-math-optimizations"
cmake -DPI_BUILD=ON -DCMAKE_BUILD_TYPE=Release -DENABLE_VFPV3=ON -DENABLE_VFPV4=ON -DENABLE_NEON=ON -DWITH_TBB=ON -DBUILD_TBB=ON -DCMAKE_CXX_FLAGS=$EXTRA_BUILD_FLAGS ../
ccache make -j3
