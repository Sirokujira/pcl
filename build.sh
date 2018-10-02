#!/bin/bash
mkdir build
cd build

set -ex
export NANOFLANN_ROOT=/usr/local/include

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
  -DBUILD_examples=OFF \
  -DBUILD_tools=ON \
  -DBUILD_apps=OFF \
  -DBUILD_global_tests=ON \
  -DBUILD_tests_2d=OFF \
  -DBUILD_tests_common=OFF \
  -DBUILD_tests_features=OFF \
  -DBUILD_tests_filters=OFF \
  -DBUILD_tests_geometry=OFF \
  -DBUILD_tests_io=OFF \
  -DBUILD_tests_keypoints=OFF \
  -DBUILD_tests_octree=OFF \
  -DBUILD_tests_outofcore=OFF \
  -DBUILD_tests_people=OFF \
  -DBUILD_tests_recognition=OFF \
  -DBUILD_tests_registration=OFF \
  -DBUILD_tests_sample_consensus=OFF \
  -DBUILD_tests_segmentation=OFF \
  -DBUILD_tests_surface=OFF \
  -DBUILD_tests_visualization=OFF \
  -DBUILD_tests_kdtree=ON \
  -DBUILD_tests_search=ON \
  ..

# make -j 4
# make install
cmake --build . --config Release --target install
ctest -C Release -V

