#!/bin/bash

#Exit on any error
set -e 

LANGUAGE=$1
BUILD_TYPE=$2

PATH=$PATH:/opt/local/bin
export PATH

OPTIONS=""

# set qt version path
OPTIONS="$OPTIONS -DQT_VERSION=6"



if [ "$LANGUAGE" == "cpp11" ]; then
  echo "Building with C++11";
  OPTIONS="$OPTIONS -DCMAKE_CXX_FLAGS='-std=c++11' "
  BUILDPATH="cpp11"  
elif [ "$LANGUAGE" == "cpp14" ]; then
  echo "Building with C++14";
  OPTIONS="$OPTIONS -DCMAKE_CXX_FLAGS='-std=c++14' "
  BUILDPATH="cpp14"
fi  

#=====================================
# Color Settings:
#=====================================
NC='\033[0m'
OUTPUT='\033[0;32m'
WARNING='\033[0;93m'

if [ "$BUILD_TYPE" == "release" ]; then
    export BUILD_TYPE=Release
    export BUILD_TYPE_L=release
else
    export BUILD_TYPE=Debug
    export BUILD_TYPE_L=debug
fi
