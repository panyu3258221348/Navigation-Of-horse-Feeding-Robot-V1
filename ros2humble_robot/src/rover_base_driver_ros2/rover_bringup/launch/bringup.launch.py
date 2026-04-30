from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('chassis_model', default_value='MiniUGV-20A',
                          description='set rover mobile robot model'),
    DeclareLaunchArgument('imu_type', default_value='onboard_imu',
                          description='IMU Model'),
]

def generate_launch_description():
    pkg_rover_bringup = get_package_share_directory('rover_bringup')
    pkg_rover_description = get_package_share_directory('rover_description')

    # Launch files
    rover_base_driver_launch_file = PathJoinSubstitution(
        [pkg_rover_bringup, 'launch/include', 'rover_base_driver.launch.py'])

    rover_imu_launch_file = PathJoinSubstitution(
        [pkg_rover_bringup, 'launch/include', 'rover_imu_driver.launch.py'])


    rover_base_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_base_driver_launch_file),
        condition= LaunchConfigurationNotEquals('imu_type', 'onboard_imu'),
        launch_arguments={
            'publish_imu': 'false'
        }.items())

    rover_base_driver_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_base_driver_launch_file),
        condition= LaunchConfigurationEquals('imu_type', 'onboard_imu'),
        launch_arguments={
            'publish_imu': 'true'
        }.items())


    rover_description_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_rover_description, 'launch', 'description.launch.py']),
        launch_arguments={
            # 'chassis_model': 'MiniUGV-10A'
        }.items())

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        rover_description_launch,
        rover_base_driver_launch,
        rover_base_driver_imu_launch,
    ])
    