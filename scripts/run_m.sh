#!/bin/bash

if [ $# -eq 0 ]; then
  echo "Usage: ./run.sh [Input filename] [Output filename]"
  exit 1
fi
cd ..
for i in `seq 1 20`;
do
	time ./build-release/path-stencil example-scenes/scene$1.xml output_images/scene$1_$i.png
done
 
