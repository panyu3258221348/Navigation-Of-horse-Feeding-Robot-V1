import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name="sensor2_rot_tf",
            arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679',
                       'rotated_camera_infra1_optical_frame', 'camera_infra1_optical_frame']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name="sensor2_rot_tf_rgb",
            arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679',
                       'rotated_camera_infra1_optical_frame', 'camera_color_optical_frame']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name="velo2cam_tf",
            arguments=['0.1', '0', '-0.18', '-3.1415926', '0', '0',
                       'rotated_camera_infra1_optical_frame', 'base_scan']
        ),
    ])
