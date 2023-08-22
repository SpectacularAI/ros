import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def readCameraCalibrationFromDevice():
    import math
    import depthai
    import numpy as np
    def matrix_to_quaternion(matrix):
        trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
        if trace > 0:
            S = math.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (matrix[2][1] - matrix[1][2]) / S
            qy = (matrix[0][2] - matrix[2][0]) / S
            qz = (matrix[1][0] - matrix[0][1]) / S
        elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
            S = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2
            qw = (matrix[2][1] - matrix[1][2]) / S
            qx = 0.25 * S
            qy = (matrix[0][1] + matrix[1][0]) / S
            qz = (matrix[0][2] + matrix[2][0]) / S
        elif matrix[1][1] > matrix[2][2]:
            S = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2
            qw = (matrix[0][2] - matrix[2][0]) / S
            qx = (matrix[0][1] + matrix[1][0]) / S
            qy = 0.25 * S
            qz = (matrix[1][2] + matrix[2][1]) / S
        else:
            S = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2
            qw = (matrix[1][0] - matrix[0][1]) / S
            qx = (matrix[0][2] + matrix[2][0]) / S
            qy = (matrix[1][2] + matrix[2][1]) / S
            qz = 0.25 * S

        return np.array([qx, qy, qz, qw])

    def toStaticParams(imuToCam):
        q = matrix_to_quaternion(imuToCam[0:3, 0:3])
        CM_TO_M = 0.01
        # x y z qx qy qz qw frame_id child_frame_id
        return [str(i * CM_TO_M) for i in imuToCam[:3, 3].tolist()] + [str(i) for i in q.tolist()]
    with depthai.Device() as device:
        calibData = device.readCalibration()
        imuToCam0 = np.array(calibData.getImuToCameraExtrinsics(depthai.CameraBoardSocket.CAM_B))
        cam0ToCam1 = np.array(calibData.getCameraExtrinsics(depthai.CameraBoardSocket.CAM_B, depthai.CameraBoardSocket.CAM_C))
        imuToCam1 = np.matmul(cam0ToCam1, imuToCam0)
        device.close()
        import time
        time.sleep(3) # Ensure device is free to be used
        return (
            toStaticParams(imuToCam0),
            toStaticParams(imuToCam1)
        )


def launch_setup(context, *args, **kwargs):
    cam0_frame, cam1_frame = readCameraCalibrationFromDevice()

    log_level = 'info'
    if(context.environment.get('DEPTHAI_DEBUG')=='1'):
        log_level='debug'

    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')

    params_file = LaunchConfiguration("params_file")
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    name = LaunchConfiguration('name').perform(context)
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')

    return [
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, 'urdf_launch.py')),
            launch_arguments={'tf_prefix': name,
                              'camera_model': camera_model,
                              'base_frame': name,
                              'parent_frame': parent_frame,
            }.items()),

        ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        parameters=[params_file],
                    )
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output="both",
        ),
        Node(package="tf2_ros", executable="static_transform_publisher", arguments=cam0_frame + ["frame_imu", "frame_cam0"]),
        Node(package="tf2_ros", executable="static_transform_publisher", arguments=cam1_frame + ["frame_imu", "frame_cam1"]),
        Node(
            package='spectacularai_ros2', executable='vislam',
            parameters=[
                {"imu_frame_id": "frame_imu"},
                {"world_frame_id": "world"},
                {"cam0_frame_id": "frame_cam1"},
                {"cam1_frame_id": "frame_cam0"},
                {"depth_scale": 1.0/1000.0}, # Depth map values are multiplied with this to get distance in meters
                {"camera_input_type": "stereo_depth_features"},
                {"recording_folder": LaunchConfiguration('recording_folder').perform(context)},
                {"enable_mapping": True},
                {"enable_occupancy_grid": True},
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
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("recording_folder", default_value=""),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("params_file", default_value='spectacularai_ros2/launch/oak_d.yaml'),
        DeclareLaunchArgument("use_rviz", default_value='true'),
        DeclareLaunchArgument("rviz_config", default_value='spectacularai_ros2/launch/oak_d.rviz')
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )



