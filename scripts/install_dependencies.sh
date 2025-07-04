#!/bin/bash
sudo apt install python3-rosdep || sudo apt install python-rosdep
if [ ! -d "src" ]
then
    echo "This script should be run in a colcon workspace. Exiting."
    exit -1
fi

if [ ! -d "depends" ]
then
    mkdir depends
fi
cd depends
touch COLCON_IGNORE

# SpaceVecAlg
git clone --recursive https://github.com/jrl-umi3218/SpaceVecAlg
cd SpaceVecAlg
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF ..
make -j
sudo make install

cd ../..

# RBDyn
git clone --recursive https://github.com/jrl-umi3218/RBDyn
cd RBDyn
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF ..
make -j
sudo make install

cd ../..

# mc_rbdyn_urdf
git clone --recursive https://github.com/jrl-umi3218/mc_rbdyn_urdf
cd mc_rbdyn_urdf
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF ..
make -j
sudo make install

cd ../..

# Rebuild the library cache
sudo ldconfig
