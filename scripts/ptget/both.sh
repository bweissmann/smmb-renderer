#!/bin/bash

rm scene$1*
./get.sh $1
python concat_images.py scene$1 $2
