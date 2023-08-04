# Spectacular AI SDK ROS integration

## Supported platforms

* ROS2 Humble on Linux

## Dependencies

ROS2 Hubmle framework:
* https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html

DepthAI ROS
* ~~https://github.com/luxonis/depthai-ros~~
* Requires modifications to depthai_examples/src/stereo_inertial_publisher.cpp, so it publishes features from left camera
    * Temporarily use https://github.com/Bercon/depthai-ros/tree/stereo_inertial_with_feature_tracker

Spectacular AI SDK

## Build

Make sure to have the custom DepthAI ROS installed ^

From the repository root, run:
```
scripts/build_all.sh -DspectacularAI_DIR=<install dir here>
```

## Run

```
source ./spectacularai_ros2/install/setup.bash && ros2 launch ./spectacularai_ros2/launch/stereo_inertial.py
```
