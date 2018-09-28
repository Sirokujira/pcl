set BOOST_ROOT=%PCL_ROOT%\3rdParty\Boost
set FLANN_ROOT=%PCL_ROOT%\3rdParty\FLANN
set EIGEN_ROOT=%PCL_ROOT%\3rdParty\Eigen\eigen3
set QHULL_ROOT=%PCL_ROOT%\3rdParty\Qhull
set VTK_DIR=%PCL_ROOT%\3rdParty\VTK
rem add custom module
set NANOFLANN_ROOT=E:\nanoflann

rem call 

cd %~dp0
mkdir build
pushd build

rem 実行するテスト項目のみ ON に設定する
rem 64 bit
rem cmake -G"Ninja" ^
cmake -G"Visual Studio 15 2017 Win64" ^
rem -DCMAKE_C_COMPILER=cl.exe ^
rem -DCMAKE_CXX_COMPILER=cl.exe ^
-DBUILD_tracking=OFF ^
-DBUILD_apps=ON ^
-DWITH_NANOFLANN=ON ^
-DPCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32=ON ^
-DPCL_BUILD_WITH_FLANN_DYNAMIC_LINKING_WIN32=ON ^
-DPCL_NO_PRECOMPILE=OFF ^
-DBUILD_global_tests=ON ^
-DBUILD_tests_2d=OFF ^
-DBUILD_tests_common=OFF ^
-DBUILD_tests_features=OFF ^
-DBUILD_tests_filters=OFF ^
-DBUILD_tests_geometry=OFF ^
-DBUILD_tests_io=OFF ^
rem -DBUILD_tests_kdtree=OFF ^
-DBUILD_tests_keypoints=OFF ^
-DBUILD_tests_octree=OFF ^
-DBUILD_tests_outofcore=OFF ^
-DBUILD_tests_people=OFF ^
-DBUILD_tests_recognition=OFF ^
-DBUILD_tests_registration=OFF ^
-DBUILD_tests_sample_consensus=OFF ^
rem -DBUILD_tests_search=OFF ^
-DBUILD_tests_segmentation=OFF ^
-DBUILD_tests_surface=OFF ^
-DBUILD_tests_visualization=OFF ^
-DBUILD_tests_kdtree=ON ^
-DBUILD_tests_search=ON ^
-DGTEST_ROOT=E:/googletest/googletest ^
..

rem msbuild PCL.sln /p:Configuration=Debug /p:Platform="X64" /target:build /filelogger
rem msbuild PCL.sln /p:Configuration=Debug /p:Platform="Any CPU" /filelogger
rem cmake --build . --config Release --target build
cmake --build . --config Release
ctest -C Release -V
popd
