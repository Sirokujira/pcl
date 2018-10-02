set VCPKG_DIR=E:\vcpkg
set PLATFORM=x64
set NINJA_DIR=E:\mytools
set GENERATOR=Ninja

rem before Install VC2017++(not Console?)
rem cd %VCPKG_DIR%
rem git pull
rem echo.set(VCPKG_BUILD_TYPE release)>> %VCPKG_DIR%\triplets\%PLATFORM%-windows.cmake
rem .\bootstrap-vcpkg.bat
rem vcpkg version
rem cd %~dp0

vcpkg install ^
boost-system ^
boost-filesystem ^
boost-thread ^
boost-date-time ^
boost-iostreams ^
boost-chrono ^
boost-asio ^
boost-dynamic-bitset ^
boost-foreach ^
boost-graph ^
boost-interprocess ^
boost-multi-array ^
boost-ptr-container ^
boost-random ^
boost-signals2 ^
eigen3 ^
flann ^
qhull ^
gtest ^
--triplet %PLATFORM%-windows

vcpkg list
set PATH=%VCPKG_DIR%\installed\%PLATFORM%-windows\bin;%PATH%
if not exist %NINJA_DIR%\ mkdir %NINJA_DIR%
cd %NINJA_DIR%
if not exist ninja.exe powershell -Command wget https://github.com/ninja-build/ninja/releases/download/v1.8.2/ninja-win.zip
if not exist ninja.exe 7z x ninja-win.zip
set PATH=%NINJA_DIR%;%PATH%

cd %~dp0
mkdir build
pushd build
rem cmake -G"Visual Studio 15 2017 Win64" ^
rem cmake -G"%GENERATOR%" ^
cmake -G"Ninja" ^
-DCMAKE_C_COMPILER=cl.exe ^
-DCMAKE_CXX_COMPILER=cl.exe ^
-DCMAKE_TOOLCHAIN_FILE=%VCPKG_DIR%\scripts\buildsystems\vcpkg.cmake ^
-DVCPKG_APPLOCAL_DEPS=OFF ^
-DBUILD_tracking=ON ^
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
rem 実行するテスト項目のみ ON に設定する
-DBUILD_tests_kdtree=ON ^
-DBUILD_tests_search=ON ^
..
cmake --build . --config Release
ctest -C Release -V
popd
