#!/bin/bash
set -e
set -o pipefail
source CI/ci-linux-prepare.sh

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Basic configuration details:"
echo "======================================================================"
echo -e "${NC}"

echo "Compiler:     $COMPILER"
echo "Options:      $OPTIONS"
echo "Language:     $LANGUAGE"
echo "Make Options: $OPTIONS"
echo "BuildPath:    $BUILDPATH"
echo "Path:         $PATH"
echo "Language:     $LANGUAGE"

if [ "$VECTORCHECKS" == "yes" ]; then
  echo -e "${OUTPUT}"
  echo ""
  echo "======================================================================"
  echo "Building $BUILD_TYPE version with vectorchecks enabled"
  echo "======================================================================"
  echo -e "${NC}"
fi

if [ ! -d build-$BUILDPATH ]; then
  mkdir build-$BUILDPATH
fi

cd build-$BUILDPATH

cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DOPENMESH_BUILD_UNIT_TESTS=TRUE $OPTIONS ../

if [ "$IWYU" == "yes" ]; then
  # do iwyu check
  if echo $(iwyu --version) | grep -q "0.11"
  then
    # support older tool version
    iwyu_tool -j 4 -p . -- \
    --mapping_file=/usr/share/include-what-you-use/gcc.libc.imp \
    --mapping_file=/usr/share/include-what-you-use/clang-6.intrinsics.imp \
    | tee iwyu.dump
  else
    # current tool version
    iwyu_tool -j 4 -p . -- \
    -Xiwyu --mapping_file=/usr/share/include-what-you-use/gcc.libc.imp \
    -Xiwyu --mapping_file=/usr/share/include-what-you-use/clang-6.intrinsics.imp \
    | tee iwyu.dump
  fi
else
  # build it
  make $MAKE_OPTIONS

  # build unittests
  make  $MAKE_OPTIONS unittests

  ldd Unittests/unittests


  # Creating System Library folder to contain all dependend libraries to run OpenFlipper
  if [ ! -d systemlib ]; then
    echo "Creating systemlib folder"
    mkdir systemlib
  fi

  echo "Copying all required libraries of OpenMesh to the systemlib directory"
  if [ "$BUILD_TYPE" == "release" ]; then
    ldd Build/lib/libOpenMeshCore.so.11.0 | grep "=> /" | awk '{print $3}' | xargs -I '{}' cp -v '{}' systemlib
  else
    ldd Build/lib/libOpenMeshCored.so.11.0 | grep "=> /" | awk '{print $3}' | xargs -I '{}' cp -v '{}' systemlib
  fi
fi

cd ..
