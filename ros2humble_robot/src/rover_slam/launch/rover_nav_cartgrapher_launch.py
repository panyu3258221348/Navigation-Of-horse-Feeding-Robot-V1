

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('rover_slam'),
            'map',
            'map.yaml'))

    param_file_name = 'nav2_params_cart_odom.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('rover_slam'),
            'param',
            param_file_name))
        # 定义外部launch文件路径
    nav_pkg_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(get_package_share_directory(
            'rover_slam'), 'launch')
    )
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('rover_slam'),
        'config',
        'cart_nav.rviz')
    pointcloud_to_laserscan_pkg_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(get_package_share_directory(
            'pointcloud_to_laserscan'), 'launch')
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # 启动 pointcloud_to_scan.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [pointcloud_to_laserscan_pkg_dir, '/pointcloud_to_scan.launch.py']
            ),
        ),
        # 启动 Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav_pkg_dir, '/movebase_cart_odom_launch.py']
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])