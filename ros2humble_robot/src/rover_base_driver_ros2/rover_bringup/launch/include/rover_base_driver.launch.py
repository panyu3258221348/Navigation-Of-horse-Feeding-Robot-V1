from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('publish_imu', default_value='false', description='Publish IMU'),
]

def generate_launch_description():
    pkg_rover_base = get_package_share_directory('rover_driver')

    # Launch files
    rover_base_driver_launch_file = PathJoinSubstitution(
        [pkg_rover_base, 'launch', 'rover_base_driver.launch.py'])

    rover_base_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rover_base_driver_launch_file]),
        launch_arguments={
            'publish_imu': LaunchConfiguration('publish_imu'),
            'pub_odom_tf': 'false',
            'linear_scale': '1.0',
            'angular_scale': '1.0'
        }.items())

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        rover_base_driver_launch
    ])
    