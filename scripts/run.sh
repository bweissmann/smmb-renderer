#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Usage: ./run.sh [Input filename] [Output filename]"
  exit 1
fi
cd ..
time ./build-release/path-stencil example-scenes/$1.xml output_images/$1.png
