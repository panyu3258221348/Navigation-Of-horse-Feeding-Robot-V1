import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    usart_port_name = LaunchConfiguration('usart_port_name', default='/dev/ttyS6')
    return LaunchDescription([
        DeclareLaunchArgument(
            'usart_port_name',
            default_value=usart_port_name,
            description='serial port for Base'),
       
        Node(
            package='rover_link',
            executable='offboard_imu',
            name='offboard_imu',
            output='screen',
            parameters=[{'usart_port_name': usart_port_name}],
           
        ),    

       
    ])

