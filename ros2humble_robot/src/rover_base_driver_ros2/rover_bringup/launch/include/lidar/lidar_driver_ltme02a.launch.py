from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals


ARGUMENTS = [
    DeclareLaunchArgument('chassis_model', default_value='MiniUGV-20A',
                          description='Lidar topic name'),
    DeclareLaunchArgument('lidar_topic_name', default_value='scan',
                          description='Lidar topic name'),
    DeclareLaunchArgument('lidar_frame_id', default_value='laser_link',
                          description='Lidar frame id'),
]

def generate_launch_description():

    ltme02a_node = Node(
        package='ltme_node',
        executable='ltme_node',
        name='ltme_node',
        output='screen',
        remappings=[('scan', LaunchConfiguration('lidar_topic_name'))],
        parameters=[
        { "device_model": "LTME-02A" },

        # IP address of LTME-02
        { "device_address": "192.168.10.160" },

        # Enforce specific transport mode used by the device to stream measurement data.
        # If set to "normal" or "oob", device settings will be changed to selected mode in case of a mismatch.
        # Available options are:
        # * none: don't enforce any mode and use device's current configuration
        # * normal: enforce normal (in-band) mode, i.e. TCP connection for command interaction is reused for data streaming
        # * oob: enforce out-of-band mode, where data are streamed through a dedicated UDP channel
        # { "enforced_transport_mode": "none" },
        
        # Frame ID used by the published LaserScan messages
        { "frame_id": LaunchConfiguration('lidar_frame_id') },

        # LTME-02 can be configured with different scan frequencies, ranging from 10 Hz to 30 Hz with 5 Hz increments.
        # The driver automatically queries device for its frequency upon connection and setup LaserScan parameters accordingly.
        # If for some reason this doesn't work for you (e.g., a device with outdated firmware),
        # this parameter can be used to override automatic detection and manually specify a correct frequency value.
        # { "scan_frequency_override": "15" },

        # Start and end angle of published scans
        # As LTME-02 has an FOV of 270 degrees, the minimum allowed value for angle_min is -2.356 (about -3 * pi / 4), and the maximum allowed value for angle_max is 2.356 (about 3 * pi / 4)
        { "angle_min": -2.35619445 },
        { "angle_max": 2.35619445 },

        # Range of angle for which data should be excluded from published scans
        # Leave these two parameters commented out if a full 270 degree FOV is desired
        # { "angle_excluded_min": -0.785 },
        # { "angle_excluded_max": 0.785 },

        # Minimum and maximum range value of published scans
        # Defaults to 0.05 and 30 respectively if not specified
        # { "range_min": 0.05 },
        # { "range_max": 30 },

        # Number of neighboring measurements to be averaged
        # Averaging reduces jitter but angular resolution will also decrease by the same factor
        # { "average_factor": 2 },

        # Adjust how data post-processing stage will filter scan artifacts caused by veiling effect
        # Valid range is between 0 and 100 inclusive, larger value leads to more aggressive filtering
        # { "shadow_filter_strength": 50 },

        # Adjust sensitivity level of the rangefinder subsystem
        # Negative value corresponds to decreased sensitivity while positive value means enhanced sensitivity
        # Only integers between -20 and 10 (inclusive) are allowed
        # { "receiver_sensitivity_boost": 0 },
        ]
    )

    tf2_miniugv_20a_node = Node(
        condition=LaunchConfigurationEquals('chassis_model', 'MiniUGV-20A'),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=["0.04", "0", "0.138", "0", "0", "0", "base_link", "laser_link"],
    )

    tf2_miniugv_50a_node = Node(
        condition=LaunchConfigurationEquals('chassis_model', 'LAUGV-50A'),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=["0.15", "0", "0.57", "0", "0", "0", "base_link", "laser_link"],
    )

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        ltme02a_node,
        tf2_miniugv_20a_node,
        tf2_miniugv_50a_node
    ])
