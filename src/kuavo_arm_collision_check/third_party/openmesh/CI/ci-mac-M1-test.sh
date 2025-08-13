#!/bin/bash
source CI/ci-mac-M1-prepare.sh

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

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Building $BUILD_TYPE version unittests"
echo "======================================================================"
echo -e "${NC}"


if [ ! -d build-$BUILD_TYPE_L-$BUILDPATH-Vector-Checks ]; then
  mkdir build-$BUILD_TYPE_L-$BUILDPATH-Vector-Checks
fi

cd build-$BUILD_TYPE_L-$BUILDPATH-Vector-Checks

#build the unit tests
make unittests

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Running unittests $BUILD_TYPE version with vectorchecks enabled"
echo "======================================================================"
echo -e "${NC}"

cd Unittests

#execute tests
./unittests --gtest_color=yes --gtest_output=xml:./report.xml

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Running unittests $BUILD_TYPE version with minimal vector type"
echo "======================================================================"
echo -e "${NC}"

./unittests_customvec --gtest_color=yes --gtest_output=xml:./report-customvec.xml

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Running unittests $BUILD_TYPE version with double vector type"
echo "======================================================================"
echo -e "${NC}"

#execute tests
./unittests_doublevec --gtest_color=yes --gtest_output=xml:./report-doublevec.xml

cd ..
cd ..
