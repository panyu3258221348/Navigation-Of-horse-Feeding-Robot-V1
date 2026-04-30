from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument('chassis_model', default_value='MiniUGV-20A',
                          description='Lidar topic name'),
    DeclareLaunchArgument('imu_topic_name', default_value='imu/data',
                          description='IMU topic name'),
    DeclareLaunchArgument('imu_frame_id', default_value='imu_link',
                          description='IMU frame id'),
]

def generate_launch_description():

    pkg_rover_bringup = get_package_share_directory('rover_bringup')

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[PathJoinSubstitution([pkg_rover_bringup, 'launch/include/imu/imu_driver_onboard_imu_filter.yaml'])],
        remappings=[
            ("imu/data", LaunchConfiguration("imu_topic_name")),
            ("imu/data_raw", "imu/onboard_imu"),
        ],
    )

    tf2_miniugv_20a_node = Node(
        condition= LaunchConfigurationEquals('chassis_model', "MiniUGV-20A"),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
         arguments=["0", "0", "0.05", "0", "0", "0", "base_link", "imu_link"],
    )

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        tf2_miniugv_20a_node,
        imu_filter_node,
    ])