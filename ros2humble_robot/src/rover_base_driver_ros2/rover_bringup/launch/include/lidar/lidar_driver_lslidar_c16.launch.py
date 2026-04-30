#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals


ARGUMENTS = [
    DeclareLaunchArgument('chassis_model', default_value='MiniUGV-20A-3D',
                          description='Lidar topic name'),
    DeclareLaunchArgument('lidar_topic_name', default_value='scan',
                          description='Lidar topic name'),
    DeclareLaunchArgument('lidar_frame_id', default_value='laser_link',
                          description='Lidar frame id'),
]

def generate_launch_description():
    pkg_lslidar_driver = get_package_share_directory('lslidar_driver')

    lslidar_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        namespace='',
        emulate_tty=True,
        parameters=[PathJoinSubstitution([pkg_lslidar_driver, 'params', 'lslidar_c16.yaml'])],
        remappings=[('scan', LaunchConfiguration('lidar_topic_name'))],
    )

    tf2_miniugv_20a_3d_node = Node(
        condition=LaunchConfigurationEquals('chassis_model', 'MiniUGV-20A-3D'),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=["0.0525", "0", "0.354", "0", "0", "0", "base_link", "laser_link"],
    )

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        lslidar_node,
        tf2_miniugv_20a_3d_node
    ])