import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    # 获取所有包的路径
    pkg_rover_bringup = get_package_share_directory('rover_bringup')
    pkg_camera = get_package_share_directory('realsense_mutile')
    pkg_pcl_filter_bringup = get_package_share_directory('point_cloud_filter')
    pkg_twist_mux_bringup = get_package_share_directory('twist_mux')
    pkg_model_status_bringup = get_package_share_directory('rover_link')
    pkg_rtk_bringup = get_package_share_directory('nmea_navsat_driver')
    pkg_gnss_imu_sim = get_package_share_directory('gnss_imu_sim')
    pkg_imu_amend = get_package_share_directory('imu_amend')

    # 声明启动参数 - 是否启动IMU修正
    declare_imu_amend_arg = DeclareLaunchArgument(
        'use_imu_amend',
        default_value='False',  # 默认启动IMU修正
        description='Whether to launch the IMU amend node'
    )

    # 从环境变量获取参数
    rover_chassis_model = os.getenv('LINGAO_MODEL', "MiniUGV-20A")
    rover_imu_type = os.getenv('LINGAO_IMU', "onboard_imu")
    use_imu_amend = LaunchConfiguration('use_imu_amend')

    # Launch文件路径
    rover_bringup_launch_file = PathJoinSubstitution(
        [pkg_rover_bringup, 'launch', 'bringup.launch.py'])

    rover_lidar_launch_file = PathJoinSubstitution(
        [pkg_rover_bringup, 'launch/include', 'rover_lidar_driver.launch.py'])
    
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
    
    imu_driver_launch_file = PathJoinSubstitution(
        [pkg_gnss_imu_sim, 'launch', 'imu_driver_launch.py'])
    
    imu_amend_launch_file = PathJoinSubstitution(
        [pkg_imu_amend, 'launch', 'imu_amend.launch.py'])

    # 启动动作定义
    rover_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_bringup_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
            'imu_type': rover_imu_type,
        }.items())
    
    rover_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_lidar_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    
    rover_pcl_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rover_pcl_filter_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    
    rover_twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_mux_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    
    rover_model_status_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(model_status_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    
    rover_rtk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtk_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    
    imu_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_driver_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items())
    
    # 条件性启动IMU修正
    imu_amend_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_amend_launch_file),
        launch_arguments={
            'chassis_model': rover_chassis_model,
        }.items(),
        condition=IfCondition(use_imu_amend)  # 根据参数决定是否启动
    )

    return LaunchDescription([
        declare_imu_amend_arg,      # 声明参数
        rover_bringup_launch,      # 底盘通信
        rover_lidar_launch,        # 激光雷达
        imu_driver_launch,         # IMU驱动
        imu_amend_launch,          # IMU修正(条件性启动)
        rover_pcl_filter_launch,   # 点云过滤
        # rover_twist_mux_launch,    # 速度多路复用
        rover_model_status_launch, # 状态监控
        rover_rtk_launch,          # RTK定位
    ])
