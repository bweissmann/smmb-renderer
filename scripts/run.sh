#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Usage: ./run.sh [Input filename] [Output filename]"
  exit 1
fi
cd ..
time ./build-release/path-stencil src/example-scenes/scene$1.xml output_images/scene$1.png