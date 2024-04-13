#!/bin/sh
apk add --update clang make cmake git

CC=clang
CXX=clang++
TRAJOPT_COMMIT=fee4a12f90abcf96d2c09a6ef737cd29b8b706d3

wget -c https://github.com/SleipnirGroup/TrajoptLib/archive/$TRAJOPT_COMMIT.tar.gz -O - | tar -xz
mkdir TrajoptLib-$TRAJOPT_COMMIT/build
cd TrajoptLib-$TRAJOPT_COMMIT/build
rm ../CMakeLists.txt
cp ../../trajoptlib-CMakeLists.txt ../CMakeLists.txt
cmake -DOPTIMIZER_BACKEND=casadi -DBUILD_TESTING=OFF ..
make -j$(nproc)
make install
cd ../..