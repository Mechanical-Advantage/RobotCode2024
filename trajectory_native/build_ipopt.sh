#!/bin/sh
apk add --update clang make gfortran blas-dev lapack-dev

CC=clang
CXX=clang++
MUMPS_VERSION="3.0.5"
IPOPT_VERSION="3.14.14"

wget -c https://github.com/coin-or-tools/ThirdParty-Mumps/archive/refs/tags/releases/$MUMPS_VERSION.tar.gz -O - | tar -xz
cd ThirdParty-Mumps-releases-3.0.5
./get.Mumps
./configure
make -j$(nproc)
make install
cd ..

wget -c https://github.com/coin-or/Ipopt/archive/refs/tags/releases/$IPOPT_VERSION.tar.gz -O - | tar -xz
mkdir Ipopt-releases-$IPOPT_VERSION/build
cd Ipopt-releases-$IPOPT_VERSION/build
../configure
make -j$(nproc)
make install
cd ../..