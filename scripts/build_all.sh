#!/bin/bash

set -eux

: "${BUILD_TYPE:=Release}"
: "${CC:=clang}"
: "${CXX:=clang++}"
: "${BUILD_SLAM:=ON}"
: "${USE_CCACHE:=ON}"

ROOT=$(pwd)

export CC
export CXX

cd $ROOT/spectacularai_ros2
# sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build --cmake-args "$@"
