import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,ExecuteProcess, RegisterEventHandler, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import ComposableNodeContainer, Node
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration('name').perform(context)
    reset_pwm = ExecuteProcess(
        cmd=[['busybox devmem 0x20320180 32 0x00000000']],
        shell=True
    )
    sai_slam = ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    parameters=[params_file],
                ),
                ComposableNode(
                        package="depthai_filters",
                        name="feature_overlay_rgb",
                        plugin="depthai_filters::FeatureTrackerOverlay",
                        remappings=[('rgb/preview/image_raw', name + '/right/image_rect'),
                                    ('feature_tracker/tracked_features', name + '/right_rect_feature_tracker/tracked_features')]
                    ),
                ComposableNode(
                    package='spectacularai_ros2',
                    plugin='spectacularAI::ros2::Node',
                    parameters=[
                        # Camera extrinsics
                        {"cam0_optical_frame_id": name + "_right_camera_optical_frame"},
                        {"cam1_optical_frame_id": name + "_left_camera_optical_frame"},
                        {"base_link_frame_id": LaunchConfiguration('parent_frame').perform(context)},
                        {"depth_scale": 1.0/1000.0}, # Depth map values are multiplied with this to get distance in meters
                        {"camera_input_type": "stereo_depth_features"},
                        {"recording_folder": LaunchConfiguration('recording_folder').perform(context)},
                        {"enable_mapping": True},
                        {"enable_occupancy_grid": True},
                        {"separate_odom_tf": True},
                        {"device_model": "RAE"} # Used to fetch imu to camera transformation
                    ],
                    remappings=[
                        ('input/imu', name + '/imu/data'),
                        ('input/cam0/image_rect', name + '/right/image_rect'),
                        ('input/cam1/image_rect', name + '/left/image_rect'),
                        ('input/cam0/camera_info', name + '/right/camera_info'),
                        ('input/cam1/camera_info', name + '/left/camera_info'),
                        ('input/depth/image', name + '/stereo/image_raw'),
                        ('input/cam0/features', name + '/right_rect_feature_tracker/tracked_features'),
                    ]
                )
            ],
            arguments=['--ros-args', '--log-level', 'info'],
            output="both",
        )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rae_description'), 'launch', 'rsp.launch.py')),
            launch_arguments={'sim': 'false'}.items()
        ),
        sai_slam,
        RegisterEventHandler(
            OnProcessStart(
                target_action=sai_slam,
                on_start=[
                    TimerAction(
                        period=15.0,
                        actions=[reset_pwm, LogInfo(msg='Resetting PWM.'),],
                    )
                ]
            )
        ),
    ]


def generate_launch_description():
    spectacular_prefix = get_package_share_directory("spectacularai_ros2")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="rae"),
        DeclareLaunchArgument("use_rviz", default_value='false'),
        DeclareLaunchArgument("recording_folder", default_value=""),
        DeclareLaunchArgument("parent_frame", default_value="base_footprint"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(spectacular_prefix, 'launch', 'rae.yaml'))
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )




