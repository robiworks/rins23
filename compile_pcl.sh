#!/bin/bash

set -eux

PCL_DOWNLOAD="https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.9.0.tar.gz"

cd $(mktemp -d)
wget $PCL_DOWNLOAD -O pcl.tar.gz
tar xvf pcl.tar.gz
cd pcl-pcl-1.9.0 && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make -j$(nproc) install
