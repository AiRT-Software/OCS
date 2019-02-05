#!/bin/bash

echo "You are going to install development tools..."

sudo apt-get install nasm ## for turbojpeg

mkdir lib
cd external-libs

read -p "Press [ENTER] to build flann"

echo "Building flann..."
cd flann-1.9.1
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make install
cd ../../

read -p "Press [ENTER] to build eigen"

echo "Building eigen..."
cd eigen-3.3.4
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
sudo make install
cd ../../

read -p "Press [ENTER] to install libboost1.62"

echo "Installing libboost..."
## lazarus dependencies
sudo apt-get install libboost-coroutine1.62
sudo apt-get install libboost-log1.62

## atreyu dependencies
sudo apt-get install libboost-program-options1.62
sudo apt-get install libboost-system1.62.0
sudo apt-get install libboost-filesystem1.62.0
sudo apt-get install libboost-thread1.62.0
sudo apt-get install libboost-date-time1.62.0
sudo apt-get install libboost-iostreams1.62.0
sudo apt-get install libboost-serialization1.62.0
sudo apt-get install libboost-chrono1.62.0
sudo apt-get install libboost-atomic1.62.0
sudo apt-get install libboost-regex1.62.0

read -p "Press [ENTER] to install libboost1.62-dev"

echo "Installing libboost-dev..."
sudo apt-get install libboost1.62-dev
sudo apt-get install libboost-system1.62-dev
sudo apt-get install libboost-filesystem1.62-dev
sudo apt-get install libboost-date-time1.62-dev
sudo apt-get install libboost-iostreams1.62-dev
sudo apt-get install libboost-coroutine1.62-dev
sudo apt-get install libboost-log1.62-dev
sudo apt-get install libboost-program-options1.62-dev

read -p "Press [ENTER] to install pcl-1.8"

echo "Installing pcl-1.8..."
sudo apt-get install libpcl-common1.8
sudo apt-get install libpcl-features1.8
sudo apt-get install libpcl-filters1.8
sudo apt-get install libpcl-io1.8
sudo apt-get install libpcl-kdtree1.8
sudo apt-get install libpcl-keypoints1.8
sudo apt-get install libpcl-ml1.8
sudo apt-get install libpcl-octree1.8
sudo apt-get install libpcl-outofcore1.8
sudo apt-get install libpcl-people1.8
sudo apt-get install libpcl-recognition1.8
sudo apt-get install libpcl-registration1.8
sudo apt-get install libpcl-sample-consensus1.8
sudo apt-get install libpcl-search1.8
sudo apt-get install libpcl-segmentation1.8
sudo apt-get install libpcl-stereo1.8
sudo apt-get install libpcl-surface1.8
sudo apt-get install libpcl-tracking1.8
sudo apt-get install libpcl-visualization1.8

read -p "Press [ENTER] to install pcl-1.8-dev"

echo "Installing pcl-1.8-dev..."
sudo apt-get install libpcl-dev  ## it will install also a lot of libboost-dev libraries
sudo apt-get install libpcl-conversions-dev
sudo apt-get install libpcl-msgs-dev

:"In ubulinux you can download directly from repositories.
These steps are for compiling pcl-1.8 from source:

read -p 'Press [ENTER] to build pcl-1.8'
echo 'Building pcl...''
cd pcl-pcl-1.8.0
mkdir build
cd build
cmake -DBUILD_tools=OFF -DBUILD_apps=OFF -DWITH_QT=OFF -DWITH_VTK=OFF -DWITH_PNG=OFF -DWITH_FZAPI=OFF -DWITH_LIBUSB=OFF -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF -DWITH_PCAP=OFF -DWITH_PXCAPI=OFF -DWITH_QHULL=OFF -DCMAKE_BUILD_TYPE=Release -DWITH_OPENGL=OFF -DCMAKE_BUILD_TYPE=Release ..
make -j2
make install 
cd ../../
"

read -p "Press [ENTER] to build librealsense2.11"

echo "Building librealsense2.11..."
cd librealsense-2.11.0/
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
uname -r  ##kernel version
sudp apt-get install libssl-dev
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install pkg-config
sudo apt-get install libgtk-3-dev
./scripts/install_glfw3.sh  ## for building graphical examples

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
./scripts/patch-realsense-ubuntu-xenial.sh  ## it will fail in ubilinux

mkdir build
cd build
cmake -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make install

read -p "Press [ENTER] to finish"


:"Old librealsense1.11 for zr300 camera:
read -p 'Press [ENTER] to install librealsense1.11'
echo 'Building librealsense...'
cd librealsense
sudo apt update && sudo apt full-upgrade
sudo apt-get install g++ pkg-config libusb-1.0-0-dev
sudo apt-get install libglfw3-dev libxrandr-dev libxi-dev  #for compiling the examples
sudo apt-get install freeglut3-dev  #for viewing the graphical examples
mkdir build
cd build
cmake -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true ..
make -j2
"
