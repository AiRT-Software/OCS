#!/bin/bash

rm -r ./lib
mkdir ./lib

echo "Compiling restc-cpp..."
cd external-libs/restc-cpp-master/
rm -r build
mkdir build
cd build
cmake -DRESTC_CPP_WITH_EXAMPLES=OFF -DRESTC_CPP_WITH_UNIT_TESTS=OFF -DRESTC_CPP_WITH_FUNCTIONALT_TESTS=OFF -DRESTC_CPP_WITH_TLS=OFF -DRESTC_CPP_WITH_ZLIB=OFF ..
make -j2  ##it generates the file in: /external-libs/restc-cpp/librestc-cpp.a
cp ../lib/librestc-cpp.a ../../../lib
cp ./generated-include/restc-cpp/config.h ../include/restc-cpp
cd ../../
read -p "Press [ENTER] to continue"

echo "Compiling zeromq..."
cd zeromq-4.2.1
rm -r build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
cp lib/* ../../../lib
cd ../../
read -p "Press [ENTER] to continue"

echo "Compiling libjpeg-turbo..."
cd libjpeg-turbo-1.5.90/
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SHARED=OFF -DENABLE_STATIC=ON ..
make
cp libturbojpeg.a ../../../lib
cd ../../
read -p "Press [ENTER] to continue"


echo "Compiling atreyu..."
cd ..
mkdir builddebug
cd builddebug
cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_EXAMPLES=ON ..
make -j1
read -p "Press [ENTER] to continue"
