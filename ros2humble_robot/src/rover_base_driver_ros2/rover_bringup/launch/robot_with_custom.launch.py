import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rover_chassis_model = os.getenv('LINGAO_MODEL', "MiniUGV-20A")
    rover_imu_type = os.getenv('LINGAO_IMU', "onboard_imu")

    pkg_rover_bringup = get_package_share_directory('rover_bringup')
    pkg_camera = get_package_share_directory('realsense_mutile')
    pkg_pcl_filter_bringup = get_package_share_directory('point_cloud_filter')
    pkg_twist_mux_bringup = get_package_share_directory('twist_mux')
    pkg_model_status_bringup = get_package_share_directory('rover_link')

    pkg_rtk_bringup=get_package_share_directory('nmea_navsat_driver')
    # Launch files
    rover_bringup_launch_file = PathJoinSubstitution(
        [pkg_rover_bringup, 'launch', 'bringup.launch.py'])

    rover_lidar_launch_file = PathJoinSubstitution(
        [pkg_rover_bringup, 'launch/include', 'rover_lidar_driver_custom.launch.py'])
    
    rover_camera_launch_file = PathJoinSubstitution(
        [pkg_camera, 'launch', 'rs_launch_stereo.py'])
    
    rover_pcl_filter_launch_file = PathJoinSubstitution(
        [pkg_pcl_filter_bringup, 'launch', 'point_cloud_filter_node_launch.py'])
    
    twist_mux_launch_file = PathJoinSubstitution(
        [pkg_twist_mux_bringup, 'launch', 'twist_mux_launch.py'])
    
    model_status_launch_file = PathJoinSubstitution(
        [pkg_model_status_bringup, 'launch', 'rover_start_model_status_launch.py'])

    
    rtk_launch_file = PathJoinSubstitution(
        [pkg_rtk_bringup, 'launch', 'nmea_serial_driver.launch.py'])
    # 启动底盘通信
    rover_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_bringup_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
            'imu_type': rover_imu_type,
        }.items())
    
    # 启动雷达
    rover_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_lidar_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    
    # 启动双目
    # rover_camare_launch =  IncludeLaunchDescription(
    #       PythonLaunchDescriptionSource(rover_camera_launch_file),
    #     launch_arguments={
    #         'chassis_model': rover_chassis_model,
    #     }.items())

    # 启动雷达过滤
    rover_pcl_filter_launch =  IncludeLaunchDescription(
          PythonLaunchDescriptionSource(rover_pcl_filter_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    # 启动多路遥控器控制器
    rover_twist_mux_launch =  IncludeLaunchDescription(
          PythonLaunchDescriptionSource(twist_mux_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    # 启动多路遥控器控制器
    rover_model_status_launch =  IncludeLaunchDescription(
          PythonLaunchDescriptionSource(model_status_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    
    # 启动RTK
    rover_rtk_launch =  IncludeLaunchDescription(
          PythonLaunchDescriptionSource(rtk_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    

    return LaunchDescription([
        rover_bringup_launch,
        rover_lidar_launch,
        rover_pcl_filter_launch,
        rover_twist_mux_launch,
        # rover_camare_launch，
        rover_model_status_launch,
        # rover_rtk_launch,
    ])
    