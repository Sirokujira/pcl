#!/bin/bash
mkdir build && cd build

set -ex

cmake \
  -DCMAKE_INSTALL_PREFIX=$PREFIX \
  -DCMAKE_BUILD_TYPE=Release \
  -DWITH_CUDA=OFF \
  -DWITH_DAVIDSDK=OFF \
  -DWITH_DSSDK=OFF \
  -DWITH_ENSENSO=OFF \
  -DWITH_FZAPI=OFF \
  -DWITH_LIBUSB=OFF \
  -DWITH_OPENGL=OFF \
  -DWITH_OPENNI=OFF \
  -DWITH_OPENNI2=OFF \
  -DWITH_PCAP=OFF \
  -DWITH_PNG=OFF \
  -DWITH_QHULL=OFF \
  -DWITH_QT=OFF \
  -DWITH_VTK=OFF \
  -DBUILD_global_tests=OFF \
  -DBUILD_examples=OFF \
  -DBUILD_tools=ON \
  -DBUILD_apps=OFF \
  ..

make -j 4
make install
