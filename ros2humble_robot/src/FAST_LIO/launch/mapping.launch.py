from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess   # ← 正确位置
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('fast_lio')
    default_cfg = os.path.join(pkg_share, 'config', 'mid360.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'fastlio.rviz')

    ld = LaunchDescription()

    # 参数声明
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('config_file', default_value=default_cfg))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('rviz_cfg', default_value=default_rviz))

    # fast_lio 原节点
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[LaunchConfiguration('config_file'),
                   {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    relay_node = Node(
        package='fast_lio',          # 脚本已安装在此包
        executable='odometry_relay.py',
        output='screen'
    )

    ld.add_action(relay_node)
  

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)


    return ld