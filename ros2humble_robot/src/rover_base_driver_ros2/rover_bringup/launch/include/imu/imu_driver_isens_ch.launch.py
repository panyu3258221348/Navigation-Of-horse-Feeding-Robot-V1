from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration

ARGUMENTS = [
    # DeclareLaunchArgument('chassis_model', default_value='MiniUGV-20A',
    #                       description='Lidar topic name'),
    DeclareLaunchArgument('imu_topic_name', default_value='/imu/data',
                          description='IMU topic name'),
    DeclareLaunchArgument('imu_frame_id', default_value='imu_link',
                          description='IMU frame id'),
]

def generate_launch_description():

    rover_imu_node = Node(
        package='rover_isens_ros',
        name='rover_imu_node',
        namespace='rover_imu',
        executable='ch_sensor_node',
        output='screen',
        parameters=[{
            "port_name": '/dev/rover_isens_imu',
            "port_baud": 921600,
            "frame_id": LaunchConfiguration('imu_frame_id'),
            "max_rate": 500,
        }],
        remappings=[
            ("/rover_imu/data", LaunchConfiguration("imu_topic_name")),
            ("/rover_imu/mag", "/imu/mag"),
        ],
        )

    tf2_miniugv_20a_node = Node(
        condition= LaunchConfigurationEquals('chassis_model', "MiniUGV-20A"),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
         arguments=["0.0525", "0", "0.24", "0", "0", "0", "base_link", "imu_link"],
    )

    tf2_miniugv_20a_3d_node = Node(
        condition=LaunchConfigurationEquals('chassis_model', 'MiniUGV-20A-3D'),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=["0.0525", "0", "0.24", "0", "0", "0", "base_link", "imu_link"],
    )

    tf2_laugv_50a_node = Node(
        condition= LaunchConfigurationEquals('chassis_model', "LAUGV-50A"),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
         arguments=["0.15", "0", "0.32", "0", "0", "0", "base_link", "imu_link"],
    )

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        tf2_miniugv_20a_3d_node,
        tf2_miniugv_20a_node,
        tf2_laugv_50a_node,
        rover_imu_node
    ])