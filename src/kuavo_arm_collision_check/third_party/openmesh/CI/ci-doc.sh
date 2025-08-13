#!/bin/bash

if [ ! -d build-release ]; then
  mkdir build-release
fi

cd build-release

cmake -DCMAKE_BUILD_TYPE=Release -DOPENMESH_BUILD_UNIT_TESTS=FALSE ../

make doc

# Copy to webserver
scp -r -P 2222 Build/share/OpenMesh/Doc/html/* gitlab@web4-info8:OpenMesh-Daily/Doc/
