#!/bin/bash
(cd ../output_images/anim; rm *.png)
(cd ../example-scenes/anim; rm *.xml; python animate.py)
./run_animation.sh
