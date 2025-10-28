#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)
cd $SCRIPT_DIR/protos

# Create directories if they don't exist
mkdir -p python csharp

# Generate Python and C# files
protoc --proto_path=/usr/include --proto_path=. --python_out=python --csharp_out=csharp *.proto

# Copy Python files to scripts/core/ros/ for backward compatibility
cp python/*_pb2.py $SCRIPT_DIR/scripts/core/ros/