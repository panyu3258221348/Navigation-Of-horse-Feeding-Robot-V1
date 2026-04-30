import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'nav.rviz')

def generate_launch_description():

    nav_rviz = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )
        # 定义外部launch文件路径
    nav_pkg_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(get_package_share_directory(
            'rover_slam'), 'launch')
    )
    livox_imu_filter_pkg_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(get_package_share_directory(
            'rover_bringup'), 'launch/include')
    )

    return LaunchDescription([
        # 启动 livox_imu_filter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [livox_imu_filter_pkg_dir, '/rover_livox_imu_driver.launch.py']
            ),
        ),

Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
              'publish_tf': False,
              'frame_id':'base_footprint',
              'odom_frame_id':'odom',
              'wait_for_transform':0.2,
              'expected_update_rate':20.0,
              'use_sim_time':False,
              # RTAB-Map's internal parameters are strings:
              'Icp/PM': True,
              'Icp/VoxelSize': '0.1',
              'Icp/PointToPlaneK': '5',
              'Icp/PointToPlaneRadius': '0',
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '20',
              'Icp/Epsilon': '0.001',
              'Icp/MaxTranslation': '1',
              'Icp/MaxCorrespondenceDistance': '0.5',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.3',
              'Icp/CorrespondenceRatio': '0.01',
              'Odom/ScanKeyFrameThr': '0.3',
              'OdomF2M/ScanSubtractRadius': '0.1',
              'OdomF2M/ScanMaxSize': '25000',
              'OdomF2M/BundleAdjustment': 'false',
              'Reg/Force3DoF':'true',  #强制 3DoF 注册：roll、pitch和z不会被估计
              'Optimizer/Slam2D':True, #强制视觉里程计仅在3DOF（x，y，theta）中跟踪车辆,增加地图的鲁棒性,配合参数：Reg/Force3DoF=true
              'wait_imu_to_init':True
            }],
            remappings=[
              ('scan_cloud', '/output_filter'),
              ('imu', '/imu/livox_data'),
              ('odom', 'icp_odom')
            ]),
            
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
              'frame_id':'base_footprint',
              'subscribe_depth':False,
              'subscribe_rgb':False,
              'subscribe_scan_cloud':True,
              'approx_sync':True,
              'wait_for_transform':0.1,
              'use_sim_time':False,
              'RGBD/ProximityMaxGraphDepth': '0',
              'RGBD/ProximityPathMaxNeighbors': '1',
              'RGBD/AngularUpdate': '0.05',
              'RGBD/LinearUpdate': '0.05',
              'RGBD/CreateOccupancyGrid': 'false',
              'RGBD/NeighborLinkRefining':'true',
              'Mem/NotLinkedNodesKept': 'true',
              'Mem/STMSize': '400',
              'Mem/LaserScanNormalK': '5',
              'Reg/Strategy': '1',
              'Icp/PM': True,
              'Icp/VoxelSize': '0.01',
              'Icp/PointToPlaneK': '5',
              'Icp/PointToPlaneRadius': '0',
              'Icp/PointToPlane': 'true',
              'Icp/Iterations': '10',
              'Icp/Epsilon': '0.001',
              'Icp/MaxTranslation': '1',
              'Icp/MaxCorrespondenceDistance': '0.5',
              'Icp/Strategy': '1',
              'Icp/OutlierRatio': '0.6',
              'Icp/CorrespondenceRatio': '0.1',
              'Rtabmap/TimeThr':'2000',
              'Rtabmap/MemoryThr': '2000',
              'Grid/ClusterRadius': '1', 
              'Grid/RangeMax': '6',
              'Grid/RayTracing': 'true',
              'Grid/Sensor': 'false',  #Gridmap is come from depth camera  or not ,default:false ,come from lidar
              'Reg/Force3DoF':'true',  #强制 3DoF 注册：roll、pitch和z不会被估计
              'Optimizer/Slam2D':True, #强制视觉里程计仅在3DOF（x，y，theta）中跟踪车辆,增加地图的鲁棒性,配合参数：Reg/Force3DoF=true
              'wait_imu_to_init':True,
              'Mem/IncrementalMemory':'False',
              'Mem/InitWMWithAllNodes':'False',
              'latch':False,
              'rtabmapviz':True,
            }],
            remappings=[
              ('scan_cloud', '/output_filter'),
              ('imu', '/imu/livox_data'),

            ]
            ), 
       
        # 启动 Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav_pkg_dir, '/movebase_icp_odom_launch.py']
            ),
        ),

        

        nav_rviz,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=nav_rviz,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown()),
                ]
            )
        )
    ])