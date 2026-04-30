#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryRelay(Node):
    def __init__(self):
        super().__init__('odometry_relay')
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.callback, 10)
        self.pub = self.create_publisher(
            Odometry, '/fastlin_odometry', 10)

    def callback(self, msg: Odometry):
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OdometryRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
