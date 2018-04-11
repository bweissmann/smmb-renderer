#!/bin/bash
cd ../build-release
qmake ../src/path-stencil.pro -spec macx-clang CONFIG+=x86_64 && /usr/bin/make qmake_all
make