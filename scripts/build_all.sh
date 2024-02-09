#!/bin/bash

set -eux

ROOT=$(pwd)

cd $ROOT/spectacularai_ros2
# sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
colcon build --cmake-args "$@" --symlink-install
