#!/bin/sh
apk add --update clang make cmake grpc-dev protobuf-dev nlohmann-json fmt-dev

CC=clang
CXX=clang++

mkdir build
cd build
cmake ..
make -j$(nproc)
cd ..