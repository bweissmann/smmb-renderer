#!/bin/bash
echo "This script doesn't work yet"
exit 1

if [ $# -eq 0 ]; then
  echo "Usage: ./run.sh [Input filename] [Output filename]"
  exit 1
fi
time ./build-path-stencil-Desktop_Qt_5_7_1_clang_64bit-Release/path-stencil Path-Tracer/example-scenes/scene$1.xml new_output/scene$1_$2.png $2 $3
