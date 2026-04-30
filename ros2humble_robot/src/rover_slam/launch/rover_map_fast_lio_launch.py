import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch


def generate_launch_description():

    # 定义外部launch文件路径
    fastlio_pkg_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(get_package_share_directory(
            'fast_lio'), 'launch')
    )

    return LaunchDescription([
        
        # 启动 fast_lio mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [fastlio_pkg_dir, '/mapping.launch.py']
            ),
        ),
        
    ])