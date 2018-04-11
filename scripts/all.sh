#!/bin/bash
echo "Building"
time ./build.sh >/dev/null 
echo "Running"
./run.sh $1