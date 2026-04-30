from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('chassis_model', default_value='MiniUGV-20A',
                          description='Lidar topic name'),
    DeclareLaunchArgument('imu_topic_name', default_value='imu/livox_data',
                          description='IMU topic name'),
    DeclareLaunchArgument('imu_frame_id', default_value='livox_frame',
                          description='IMU frame id'),
]

def generate_launch_description():

    pkg_rover_bringup = get_package_share_directory('rover_bringup')

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_rover_bringup, 'launch/include/imu/imu_driver_livox_imu_filter.yaml'])],
        remappings=[
            ("imu/data", LaunchConfiguration("imu_topic_name")),
            ("imu/data_raw", "/livox/imu"),
        ],
    )

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        imu_filter_node,
    ])