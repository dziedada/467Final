#!/bin/bash

# Get location of folder
SOFTWARE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
SOFTWARE_DIR="$(dirname "$SOFTWARE_DIR")"

# Add upstream repository as remote
echo "Setting upstream to maav/software..."
git remote add upstream git@github.com:dziedada/467Final.git

# Get submodules
echo "Initializing submodules..."
git submodule update --init --recursive

sudo apt-get update && apt-get upgrade
sudo apt-get install default-jdk

# Install dependencies
sudo apt install cmake \
                 cmake-curses-gui \
                 clang-format \
                 libyaml-cpp-dev \
                 curl \
                 libcurl4-openssl-dev \
                 ffmpeg \
                 libglew-dev \
                 libev-dev \
                 libgtkmm-3.0-dev \
                 libudev-dev \
                 libglm-dev \
                 libusb-1.0-0-dev \
                 libusb-1.0-doc \
                 libusb-1.0-0-dbg \
                 libflann1.8 \
                 python3 \
                 python3-pip \
                 libvtk6-dev \
                 cython \
                 cython3 \
                 libzmq3-dev \
                 -y # system libraries

# Install custom deps
${SOFTWARE_DIR}/scripts/install/install-cmake.sh
${SOFTWARE_DIR}/scripts/install/install-g++7.sh
${SOFTWARE_DIR}/scripts/install/install-zcm.sh
${SOFTWARE_DIR}/scripts/install/install-librealsense.sh
${SOFTWARE_DIR}/scripts/install/install-pangolin.sh
${SOFTWARE_DIR}/scripts/install/install-pcl.sh
${SOFTWARE_DIR}/scripts/install/install-opencv3.sh
${SOFTWARE_DIR}/scripts/install/install-lcm.sh
${SOFTWARE_DIR}/scripts/install/install-apriltag.sh
${SOFTWARE_DIR}/scripts/install/install-qt5.sh
