from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('imu_amend'),
        'config',
        'imu_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='imu_amend',
            executable='imu_amend_node',
            name='imu_straight_controller',
            parameters=[config],
            output='screen'
        )
    ])