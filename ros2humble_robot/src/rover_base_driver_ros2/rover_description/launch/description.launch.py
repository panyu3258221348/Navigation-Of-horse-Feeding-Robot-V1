import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('chassis_model', default_value='MiniUGV-10A',
                          description='Set rover mobile robot model'),
]

# evaluates LaunchConfigurations in context for use with xacro.process_file(). Returns a list of launch actions to be included in launch description
def evaluate_xacro(context, *args, **kwargs):
    pkg_rover_description = get_package_share_directory('rover_description')
    model_name = LaunchConfiguration('chassis_model').perform(context)
    xacro_path = os.path.join(pkg_rover_description, 'urdf', 'MiniUGV-10A.xacro')
    
    robot_state_publisher_node = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{
        'robot_description': xacro.process_file(xacro_path).toxml()
      }])

    return [robot_state_publisher_node]

def generate_launch_description():

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    return LaunchDescription([
        LaunchDescription(ARGUMENTS),
        OpaqueFunction(function=evaluate_xacro),
        joint_state_publisher_node
    ])
    