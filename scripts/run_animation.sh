#!/bin/bash

cd ..
i=0
for file in example-scenes/anim/*.xml; do
    [ -e "$file" ] || continue
    [ "example-scenes/anim/template.xml" != "$file" ] || continue
    time ./build-release/path-stencil $file output_images/anim/$i.png
    echo $file
    i=$[$i+1]
done
exit 0

