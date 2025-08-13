#!/bin/bash

# Build and install third-party libraries
# This script should be run from the third_party_libs directory

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Building third-party libraries from: $SCRIPT_DIR"

# Function to build and install a library
build_and_install() {
    local lib_dir="$1"
    local lib_name=$(basename "$lib_dir")
    
    echo "=========================================="
    echo "Building and installing: $lib_name"
    echo "=========================================="
    
    if [ ! -d "$lib_dir" ]; then
        echo "Warning: Directory $lib_dir does not exist, skipping..."
        return 0
    fi
    
    cd "$lib_dir"
    
    # Create build directory if it doesn't exist
    mkdir -p build
    cd build
    
    # Configure with cmake
    echo "Configuring $lib_name..."
    cmake .. -DCMAKE_BUILD_TYPE=Release
    
    # Build
    echo "Building $lib_name..."
    make -j$(nproc)
    
    # Install
    echo "Installing $lib_name..."
    sudo make install
    
    echo "Successfully built and installed: $lib_name"
    echo
    
    # Return to script directory
    cd "$SCRIPT_DIR"
}

# Build libraries in dependency order
# CycloneDDS should be built before CycloneDDS-CXX
echo "Starting third-party library build process..."

# Build CycloneDDS first (C library)
build_and_install "$SCRIPT_DIR/cyclonedds-0.10.2"

# Build CycloneDDS-CXX (C++ bindings, depends on CycloneDDS)
build_and_install "$SCRIPT_DIR/cyclonedds-cxx-0.10.2"

# Update library cache
echo "Updating library cache..."
sudo ldconfig

echo "=========================================="
echo "All third-party libraries built and installed successfully!"
echo "==========================================" 