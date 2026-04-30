import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'cart_map.rviz')

def generate_launch_description():

    map_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )
    pointcloud_to_laserscan_pkg_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(get_package_share_directory(
            'pointcloud_to_laserscan'), 'launch')
    )

    # 定义外部launch文件路径
    cartographer_pkg_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(get_package_share_directory(
            'rover_slam'), 'launch')
    )

    return LaunchDescription([
        
        # 启动 pointcloud_to_scan.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [pointcloud_to_laserscan_pkg_dir, '/pointcloud_to_scan.launch.py']
            ),
        ),
        # 启动 pointcloud_to_scan.launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [cartographer_pkg_dir, '/cartographer.launch.py']
            ),
        ),
        map_rviz,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=map_rviz,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown()),
                ]
            )
        )
    ])