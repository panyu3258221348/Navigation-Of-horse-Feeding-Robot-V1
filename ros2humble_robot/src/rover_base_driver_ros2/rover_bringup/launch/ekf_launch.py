from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取参数文件路径
    ekf_params_path = os.path.join(
        get_package_share_directory('rover_bringup'),
        'params',
        'ekf_localization.yaml'
    )

    # 创建EKF节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_path],
        remappings=[
            ('/odometry/filtered', '/odom'),
            ('imu/data', 'imu/data')
        ]
    )

    return LaunchDescription([
        ekf_node
    ])