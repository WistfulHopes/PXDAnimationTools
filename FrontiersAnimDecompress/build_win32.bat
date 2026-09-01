@echo off
cd %~dp0
rmdir /s /q build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
