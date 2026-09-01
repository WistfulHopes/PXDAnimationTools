@echo off
cd %~dp0
rmdir /s /q build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_CXX_COMPILER=x86_64-linux-gnu-g++
cmake --build build
