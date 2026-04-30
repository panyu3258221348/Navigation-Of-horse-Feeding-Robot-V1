from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('chassis_model', default_value='MiniUGV-20A',
                          description='Lidar topic name'),
    DeclareLaunchArgument('lidar_type', default_value='nvilidar',
                          description='Lidar Model'),
    DeclareLaunchArgument('lidar_topic_name', default_value='scan',
                          description='Lidar topic name'),
    DeclareLaunchArgument('lidar_frame_id', default_value='laser_link',
                          description='Lidar frame id'),
]

def generate_launch_description():

    pkg_rover_bringup = get_package_share_directory('livox_ros_driver2')

    # Launch files
    lidar_launch_dir = PathJoinSubstitution(
        [pkg_rover_bringup, 'launch_ROS2/'])
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_launch_dir, 'mid360_pointcloud2.py']),
        launch_arguments={
            'chassis_model': LaunchConfiguration('chassis_model'),
            'topic_name': LaunchConfiguration('lidar_topic_name'),
            'frame_id': LaunchConfiguration('lidar_frame_id'),
        }.items())

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        lidar_launch
    ])