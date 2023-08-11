# Spectacular AI SDK ROS integration

## Supported platforms

* ROS2 Humble on Linux

## Dependencies

* ROS2 Hubmle framework:
    * https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html
* DepthAI ROS
    * ~~https://github.com/luxonis/depthai-ros~~
    * Requires modifications to `depthai_examples/src/stereo_inertial_publisher.cpp`, so it publishes features from left camera
        * Temporarily use https://github.com/Bercon/depthai-ros/tree/stereo_inertial_with_feature_tracker
        * Should be converted to use Depth AI ROS driver instead once feature tracker is supported

* Spectacular AI SDK (https://www.spectacularai.com/#contact)

## Build & Run

Extract Spetacular AI SDK to `spectacularAI/` directory to root of this repo. Pick the correct version based on your CPU architecture, `../` in the beginning since build dir is `./spectacularai_ros2`:
* For x86-64: ../spectacularAI/Linux_Ubuntu_x86-64/lib/cmake/spectacularAI
* For aarch64: ../spectacularAI/Ubuntu_Aarch64/lib/cmake/spectacularAI

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

From this repository root, build with:
```
scripts/build_all.sh -DspectacularAI_DIR=../spectacularAI/Linux_Ubuntu_x86-64/lib/cmake/spectacularAI
```

Run:
```
source ./spectacularai_ros2/install/setup.bash && ros2 launch ./spectacularai_ros2/launch/stereo_inertial.py
```
