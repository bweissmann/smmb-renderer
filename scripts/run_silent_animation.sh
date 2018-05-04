#!/bin/bash

cd ..
i=0
for file in example-scenes/anim/*.xml; do
    [ -e "$file" ] || continue
    [ "example-scenes/anim/template.xml" != "$file" ] || continue
    num=ERR
    if [[ $file =~ [0-9]+ ]]; then 
        num=${BASH_REMATCH}
        echo "match: '${BASH_REMATCH}'"; 
        echo $num
    else 
        echo "no match found"; fi

    time ./build-release/path-stencil $file output_images/anim/$num > /dev/null 
    echo $file
    i=$[$i+1]
done
exit 0

