#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals


ARGUMENTS = [
    DeclareLaunchArgument('chassis_model', default_value='MiniUGV-20A',
                          description='Lidar topic name'),
    DeclareLaunchArgument('lidar_topic_name', default_value='scan',
                          description='Lidar topic name'),
    DeclareLaunchArgument('lidar_frame_id', default_value='laser_link',
                          description='Lidar frame id'),
]

def generate_launch_description():

    pkg_rover_bringup = get_package_share_directory('rover_bringup')

    nvilidar_node = LifecycleNode(
        package='nvilidar_ros2',
        executable='nvilidar_ros2_node',
        name='nvilidar_ros2_node',
        output='screen',
        namespace='',
        emulate_tty=True,
        parameters=[PathJoinSubstitution([pkg_rover_bringup, 'launch/include/lidar/lidar_driver_nvilidar_config.yaml'])],
        remappings=[('scan', LaunchConfiguration('lidar_topic_name'))],
    )

    tf2_miniugv_20a_node = Node(
        condition=LaunchConfigurationEquals('chassis_model', 'MiniUGV-20A'),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=["0.04", "0", "0.138", "0", "0", "0", "base_link", "laser_link"],
    )

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        nvilidar_node,
        tf2_miniugv_20a_node
    ])