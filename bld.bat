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
cd build
rem 64 bit
rem cmake -G"Ninja" ^
rem -DCMAKE_C_COMPILER=cl.exe ^
rem -DCMAKE_CXX_COMPILER=cl.exe ^
cmake -G"Visual Studio 15 2017 Win64" ^
-DBUILD_tracking=OFF ^
-DBUILD_apps=ON ^
-DWITH_NANOFLANN=ON ^
-DPCL_BUILD_WITH_BOOST_DYNAMIC_LINKING_WIN32=ON ^
-DPCL_BUILD_WITH_FLANN_DYNAMIC_LINKING_WIN32=ON ^
-DPCL_NO_PRECOMPILE=OFF ^
-DBUILD_global_tests=ON ^
-DBUILD_tests_kdtree=ON ^
-DBUILD_tests_search=ON ^
-DBUILD_tests_octree=OFF ^
-DGTEST_ROOT=E:/googletest/googletest ^
..

rem msbuild PCL.sln /p:Configuration=Debug /p:Platform="X64" /target:rebuild
rem msbuild PCL.sln /p:Configuration=Debug /p:Platform="Any CPU"
cmake --build . --config Release
ctest -C Release -V
