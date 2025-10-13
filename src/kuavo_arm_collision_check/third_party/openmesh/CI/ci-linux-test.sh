#!/bin/bash
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

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Building $BUILD_TYPE version unittests"
echo "======================================================================"
echo -e "${NC}"

if [ ! -d build-$BUILDPATH ]; then
  mkdir build-$BUILDPATH
fi

cd build-$BUILDPATH

#clean old cmake cache as the path might have changed
find . -name "CMakeCache.txt" -type f -delete

#just to be safe clean the test file definitions too
if [ -f CTestTestfile.cmake ]
then
	echo "Removing old CTestTestfile.cmake"
	rm CTestTestfile.cmake
fi
#just to be safe clean the test file definitions too
if [ -f DartConfiguration.tcl ]
then
	echo "Removing old DartConfiguration.tcl"
	rm DartConfiguration.tcl
fi
# Run cmake to make sure the tests are configured correctly for this system
cmake -DOPENFLIPPER_BUILD_UNIT_TESTS=TRUE -DSTL_VECTOR_CHECKS=ON $OPTIONS ../

#tell the location to the libs from build jobs
export LD_LIBRARY_PATH=$(pwd)/Build/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(pwd)/Build/systemlib:$LD_LIBRARY_PATH

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Running unittests $BUILD_TYPE version with vectorchecks enabled"
echo "======================================================================"
echo -e "${NC}"

cd Unittests

ldd ./unittests

#execute tests
./unittests --gtest_color=yes --gtest_output=xml:./report.xml

echo -e "${OUTPUT}"
echo ""
echo "======================================================================"
echo "Running unittests $BUILD_TYPE version with custom vector type"
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

pwd
ls *.xml

cd ..
cd ..
