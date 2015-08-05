#!/bin/bash

cd /root/quadtargetfsm
git pull
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_VFPV3=ON -DENABLE_VFPV4=ON -DENABLE_NEON=ON -DWITH_TBB=ON -DBUILD_TBB=ON ../
ccache make -j3
cp ../config.ini config.ini
