#!/bin/bash

set -x
set -e

# building in temporary directory to keep system clean
# use RAM disk if possible (as in: not building on CI system like Travis, and RAM disk is available)
if [ "$CI" == "" ] && [ -d /dev/shm ]; then
  TEMP_BASE=/dev/shm
else
  TEMP_BASE=/tmp
fi

# You must replace VTK build directory by setting an environment variable
if [ -z "$VTK_DIR" ]; then
  echo "VTK_DIR environment variable is not set, using default value. Please set it to the build directory of your VTK installation."
  VTK_DIR=/home/biblbrox/sources/build/vtk
fi

BUILD_DIR=$(mktemp -d -p "$TEMP_BASE" appimage-build-XXXXXX)

# make sure to clean up build dir, even if errors occur
cleanup() {
  if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
  fi
}

trap cleanup EXIT

# store repo root as variable
REPO_ROOT=$(readlink -f $(dirname $(dirname $0)))

OLD_CWD=$(readlink -f .)

# switch to build dir
echo "-- Going to build directory $BUILD_DIR"
pushd "$BUILD_DIR"

# configure build files with CMake
# we need to explicitly set the install prefix, as CMake's default is /usr/local for some reason...
VCPKG_TOOLCHAIN=/home/biblbrox/Tesis/ROS2Dashboard/thirdy/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build /home/biblbrox/Tesis/ROS2Dashboard/bin_debug --target ROS2Dashboard # -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=${VCPKG_TOOLCHAIN} -G Ninja -S ${REPO_ROOT}

cmake --install /home/biblbrox/Tesis/ROS2Dashboard/bin_debug --prefix=AppDir
cp -R $VTK_DIR/lib/* "$BUILD_DIR/AppDir/lib"

# now, build AppImage using linuxdeploy and linuxdeploy-plugin-qt
# download linuxdeploy and its Qt plugin
wget https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage
wget https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/continuous/linuxdeploy-plugin-qt-x86_64.AppImage

# make them executable
chmod +x linuxdeploy*.AppImage

# make sure Qt plugin finds QML sources so it can deploy the imported files
export QML_SOURCES_PATHS="$REPO_ROOT"/src
# initialize AppDir, bundle shared libraries for QtQuickApp, use Qt plugin to bundle additional resources, and build AppImage, all in one single command
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BUILD_DIR/AppDir/lib/"
ldd ./AppDir/bin/ROS2Dashboard
export QMAKE=/usr/bin/qmake6
./linuxdeploy-x86_64.AppImage --appdir AppDir --plugin qt --executable ./AppDir/bin/ROS2Dashboard --desktop-file /home/biblbrox/Tesis/ROS2Dashboard/ros2dashboard.desktop --icon-file /home/biblbrox/Tesis/ROS2Dashboard/res/icons/ROS2Dashboard.svg --output appimage

# move built AppImage back into original CWD
mv ROS2Dashboard*.AppImage "$OLD_CWD/dist/"
