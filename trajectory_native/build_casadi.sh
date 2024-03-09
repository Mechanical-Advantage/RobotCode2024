#!/bin/sh
apk add --update clang make cmake blas-dev lapack-dev tinyxml2-dev

CC=clang
CXX=clang++
CASADI_VERSION="3.6.4"

wget -c https://github.com/casadi/casadi/archive/refs/tags/$CASADI_VERSION.tar.gz -O - | tar -xz
mkdir casadi-$CASADI_VERSION/build
cd casadi-$CASADI_VERSION/build
cmake -DWITH_IPOPT=ON -DWITH_DEEPBIND=OFF -DWITH_BUILD_TINYXML=OFF ..
make -j$(nproc)
make install
cd ../..