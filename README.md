# Spectacular AI SDK ROS integration

## Supported platforms

* ROS2 Humble on Linux

## Dependencies

* ROS2 Humble framework:
    * https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html
* DepthAI ROS
    * ~~https://github.com/luxonis/depthai-ros~~
    * Temporarily use this fork instead of the official repository https://github.com/Bercon/depthai-ros/tree/stereo_inertial_with_feature_tracker
        * Contains modifications to `depthai_examples/src/stereo_inertial_publisher.cpp`, so it publishes features from left camera
        * Launch script should be converted to use Depth AI ROS driver instead once feature tracker is supported. Then the fork is no longer required
* Spectacular AI SDK (https://www.spectacularai.com/#contact)

## Build & Run

Extract Spectacular AI SDK somewhere, for example `spectacularAI/` and optionally install it. If you don't install it, use `-DspectacularAI_DIR=` to help CMake build find it.

### RVC2

Make sure to have the custom DepthAI ROS installed ^, for example you can build it to `~/dai_ws` folder and install with `source ~/dai_ws/install/setup.bash`:
```
mkdir -p ~/dai_ws
cd ~/dai_ws/src
git clone --branch stereo_inertial_with_feature_tracker https://github.com/Bercon/depthai-ros.git
cd ~/dai_ws
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j1 -l1" colcon build
source ~/dai_ws/install/setup.bash
```

From this repository root, assuming `x86-64` architecture and SDK in `spectacularAI/`:
```
scripts/build_all.sh -DspectacularAI_DIR=../spectacularAI/Linux_Ubuntu_x86-64/lib/cmake/spectacularAI
```

Run:
```
source ./spectacularai_ros2/install/setup.bash && ros2 launch ./spectacularai_ros2/launch/stereo_inertial.py
```
