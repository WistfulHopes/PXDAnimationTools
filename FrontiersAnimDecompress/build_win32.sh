#!/bin/bash
DIR="$(cd "$(dirname "$0")" && pwd)"
cd $DIR
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_SYSTEM_NAME=Windows -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++
cmake --build build
