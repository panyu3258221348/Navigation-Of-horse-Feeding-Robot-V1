from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

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

    # Launch files
    imu_launch_dir = PathJoinSubstitution(
        [pkg_rover_bringup, 'launch/include/imu/'])
    
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([imu_launch_dir, 'imu_driver_', 'livox_imu', '.launch.py']),
        launch_arguments={
            'chassis_model': LaunchConfiguration('chassis_model'),
            'imu_topic_name': LaunchConfiguration('imu_topic_name'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id')
        }.items())
    
    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        imu_launch
    ])