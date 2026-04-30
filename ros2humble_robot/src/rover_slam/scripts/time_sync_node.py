#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class TimeSyncNode(Node):
    def __init__(self):
        super().__init__('time_sync_node')
        
        # 设置QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 创建订阅者
        self.imu_sub = Subscriber(self, Imu, '/imu/livox_data', qos_profile=qos_profile)
        self.odom_sub = Subscriber(self, Odometry, '/fastlin_odometry', qos_profile=qos_profile)
        
        # 创建近似时间同步器
        self.ts = ApproximateTimeSynchronizer(
            [self.imu_sub, self.odom_sub],
            queue_size=10,
            slop=0.1  # 允许的时间差（秒）
        )
        self.ts.registerCallback(self.sync_callback)
        
        # 创建发布者
        self.synced_imu_pub = self.create_publisher(Imu, '/synced_imu', qos_profile=qos_profile)
        self.synced_odom_pub = self.create_publisher(Odometry, '/synced_odom', qos_profile=qos_profile)
        
        self.get_logger().info('Time Sync Node has been started')
    
    def sync_callback(self, imu_msg, odom_msg):
        # 使用较新的时间戳
        sync_time = max(imu_msg.header.stamp, odom_msg.header.stamp)
        
        # 更新IMU消息的时间戳
        synced_imu = Imu()
        synced_imu = imu_msg
        synced_imu.header.stamp = sync_time
        
        # 更新里程计消息的时间戳
        synced_odom = Odometry()
        synced_odom = odom_msg
        synced_odom.header.stamp = sync_time
        
        # 发布同步后的消息
        self.synced_imu_pub.publish(synced_imu)
        self.synced_odom_pub.publish(synced_odom)
        
        self.get_logger().debug(f'Synchronized messages at time: {sync_time.sec}.{sync_time.nanosec}')


def main(args=None):
    rclpy.init(args=args)
    node = TimeSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()