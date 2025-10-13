#!/bin/bash
source CI/ci-mac-prepare.sh

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Basic configuration details:"
echo "======================================================================"
echo -e "${NC}"

echo "Options:    $OPTIONS"
echo "BuildPath:  $BUILDPATH"
echo "Path:       $PATH"
echo "Language:   $LANGUAGE"

cmake --version

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Building $BUILD_TYPE version with vectorchecks enabled"
echo "======================================================================"
echo -e "${NC}"


if [ ! -d build-$BUILD_TYPE_L-$BUILDPATH-Vector-Checks ]; then
  mkdir build-$BUILD_TYPE_L-$BUILDPATH-Vector-Checks
fi

cd build-$BUILD_TYPE_L-$BUILDPATH-Vector-Checks

cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DOPENMESH_BUILD_UNIT_TESTS=TRUE -DSTL_VECTOR_CHECKS=ON $OPTIONS ../

#build it
make

cd ..

if [ "$BUILD_TYPE_L" == "release" ]; then

  echo -e "${OUTPUT}"
  echo ""
  echo "======================================================================"
  echo "Package creation (DMG and tarball)"
  echo "======================================================================"
  echo -e "${NC}"


  if [ ! -d build-release-$BUILDPATH ]; then
    mkdir build-release-$BUILDPATH
  fi

  cd build-release-$BUILDPATH

  cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_APPS=OFF -DCPACK_BINARY_DRAGNDROP=ON $OPTIONS ../

  #build it
  make
  make package

  cd ..

fi
