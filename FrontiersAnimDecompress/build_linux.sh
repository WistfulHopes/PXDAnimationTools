#!/bin/bash
DIR="$(cd "$(dirname "$0")" && pwd)"
cd $DIR
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
