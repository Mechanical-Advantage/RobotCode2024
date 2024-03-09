#!/bin/sh
apk add --update clang make cmake git

CC=clang
CXX=clang++
TRAJOPT_COMMIT=f6cf3d42359f6f41f311f848a4e7f51c3f88c2ca

wget -c https://github.com/SleipnirGroup/TrajoptLib/archive/$TRAJOPT_COMMIT.tar.gz -O - | tar -xz
mkdir TrajoptLib-$TRAJOPT_COMMIT/build
cd TrajoptLib-$TRAJOPT_COMMIT/build
rm ../CMakeLists.txt
cp ../../trajoptlib-CMakeLists.txt ../CMakeLists.txt
cmake -DOPTIMIZER_BACKEND=casadi -DBUILD_TESTING=OFF ..
make -j$(nproc)
make install
cd ../..