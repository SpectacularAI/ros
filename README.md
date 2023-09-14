# Spectacular AI SDK ROS integration

## Supported platforms

* ROS2 Humble on Linux

## Dependencies

* ROS2 Humble framework: https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html
* DepthAI ROS: https://github.com/luxonis/depthai-ros
* Spectacular AI SDK: https://www.spectacularai.com/#contact

## Build & Run

Extract Spectacular AI SDK somewhere, for example `spectacularAI/` and optionally install it. If you don't install it, use `-DspectacularAI_DIR=` to help CMake build find it.

From this repository root, assuming `x86-64` architecture and SDK in `spectacularAI/`:
```bash
scripts/build_all.sh -DspectacularAI_DIR=../spectacularAI/Linux_Ubuntu_x86-64/lib/cmake/spectacularAI
# OR alternatively, with a static SDK variant
# ./scripts/build_all_static.sh /PATH/TO/spectacularAI_*_static.tar.gz
```

Run:
```bash
source ./spectacularai_ros2/install/setup.bash && ros2 launch ./spectacularai_ros2/launch/oak_d.launch.py
```
