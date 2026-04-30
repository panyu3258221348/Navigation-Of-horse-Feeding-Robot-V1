#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap  # 导入服务定义
from std_msgs.msg import Empty, Bool          # 导入 Empty 和 Bool 消息
from rclpy.duration import Duration # 用于计算时间差

class CostmapSubscriberClearer(Node):

    def __init__(self):
        super().__init__('costmap_subscriber_clearer')
        self.get_logger().info('Costmap subscriber clearer node started.')

        # --- 订阅 /clear_map 话题 ---
        self.clear_map_subscription = self.create_subscription(
            Bool,
            '/clear_map',
            self.clear_map_callback,
            10) # 队列大小为 10
        self.get_logger().info(f'Subscribed to /clear_map topic.')

        # --- 创建代价地图清理服务的客户端 ---
        # 创建局部代价地图清理服务的客户端
        self.local_clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        # 等待服务可用
        while not self.local_clear_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Local costmap clear service not available, waiting...')

        # 创建全局代价地图清理服务的客户端
        self.global_clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        # 等待服务可用
        while not self.global_clear_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Global costmap clear service not available, waiting...')

        self.get_logger().info('Costmap services are available.')
        
        # 标记是否正在执行清理，避免重复触发
        self.is_clearing = False

        # --- 新增：用于定时清理的变量 ---
        self.declare_parameter('auto_clear_interval', 2.0) # 默认每 2 秒自动清理
        self.auto_clear_interval = self.get_parameter('auto_clear_interval').value
        self.last_manual_clear_time = None # 记录上次手动清理（收到 True）的时间
        self.last_auto_clear_time = self.get_clock().now() # 初始化上次自动清理时间
        
        # 创建一个定时器，用于检查是否需要自动清理
        self.auto_clear_timer = self.create_timer(
            self.auto_clear_interval,
            self.auto_clear_timer_callback
        )
        self.get_logger().info(f"自动全局地图清理间隔设置为: {self.auto_clear_interval:.1f} 秒。")


    def clear_local_costmap(self):
        """调用服务清除局部代价地图。"""
        self.get_logger().info('Requesting to clear local costmap...')
        req = ClearEntireCostmap.Request()
        future = self.local_clear_costmap_client.call_async(req)
        future.add_done_callback(self.local_costmap_response_callback)
        return future

    def clear_global_costmap(self):
        """调用服务清除全局代价地图。"""
        self.get_logger().info('Requesting to clear global costmap...')
        req = ClearEntireCostmap.Request()
        future = self.global_clear_costmap_client.call_async(req)
        future.add_done_callback(self.global_costmap_response_callback)
        return future

    def local_costmap_response_callback(self, future):
        """局部代价地图清理服务的响应回调。"""
        try:
            response = future.result()
            self.get_logger().info('Local costmap cleared successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to clear local costmap: {e}')
        # finally:
            # 清理完成后，重置标志位，以便下次触发
            # 注意：此处如果执行了自动清理，is_clearing 应该只在收到 True 时被重置。
            # 如果是自动清理，我们不需要修改 is_clearing。
            # self.is_clearing = False # 暂时注释掉，以区分手动和自动

    def global_costmap_response_callback(self, future):
        """全局代价地图清理服务的响应回调。"""
        try:
            response = future.result()
            self.get_logger().info('Global costmap cleared successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to clear global costmap: {e}')
        # finally:
            # 清理完成后，重置标志位，以便下次触发
            # self.is_clearing = False # 暂时注释掉

    def clear_map_callback(self, msg: Bool):
        """当收到 /clear_map 话题消息时的回调函数。"""
        self.get_logger().info(f'Received message on /clear_map: {msg.data}')

        # 只有当消息是 True 并且当前没有正在执行清理操作时，才触发清理
        if msg.data and not self.is_clearing:
            self.is_clearing = True # 设置标志位，表示正在进行手动清理
            self.get_logger().info('Received True on /clear_map. Triggering manual costmap clearing...')
            
            # 清除局部代价地图
            self.clear_local_costmap()
            
            # 清除全局代价地图
            self.clear_global_costmap()
            
            # 记录上次手动清理的时间
            self.last_manual_clear_time = self.get_clock().now()
            # 自动清理定时器不应该因为手动清理而重置，因为它是独立于手动信号的
            
        elif not msg.data:
            self.get_logger().info('Received False on /clear_map, no action taken.')
        elif self.is_clearing:
            self.get_logger().warn('Received True on /clear_map, but manual clearing is in progress. Ignoring.')

    def auto_clear_timer_callback(self):
        """
        定时器回调，检查是否需要自动清理全局地图。
        """
        # 只在没有收到手动清理信号，并且距离上次自动清理超过间隔时间时才执行自动清理
        
        # 检查逻辑：
        # 1. 如果 last_manual_clear_time 是 None (从未收到 True)
        #    或者
        # 2. 如果当前时间距离 last_manual_clear_time 已经超过了 auto_clear_interval
        #    并且
        # 3. 如果我们当前没有在进行手动清理 (is_clearing is False)
        #    并且
        # 4. 距离上次自动清理也已经超过了 auto_clear_interval (这防止了紧随手动清理后立刻进行自动清理)

        now = self.get_clock().now()
        
        # 计算上次自动清理的间隔
        time_since_last_auto_clear = (now - self.last_auto_clear_time).nanoseconds / 1e9

        # 只有当距离上次自动清理的时间差大于设置的间隔时，才考虑进行自动清理
        if time_since_last_auto_clear >= self.auto_clear_interval:
            # 检查是否收到过手动清理信号
            if self.last_manual_clear_time is None:
                #never received manual clear
                self.get_logger().debug(f"No manual clear received. Triggering auto clear for global map.")
                self.clear_global_costmap()
                self.last_auto_clear_time = now # 更新上次自动清理时间
            else:
                # Received manual clear, check if auto clear should be re-enabled
                time_since_last_manual_clear = (now - self.last_manual_clear_time).nanoseconds / 1e9
                if time_since_last_manual_clear >= self.auto_clear_interval and not self.is_clearing:
                    # manual clear happened longer ago than interval and not currently clearing
                    self.get_logger().debug(f"Manual clear happened {time_since_last_manual_clear:.1f}s ago. Triggering auto clear for global map.")
                    self.clear_global_costmap()
                    self.last_auto_clear_time = now # 更新上次自动清理时间
                # else:
                    # manual clear was recent, or we are still clearing, don't auto clear


def main(args=None):
    rclpy.init(args=args)

    costmap_subscriber_clearer_node = CostmapSubscriberClearer()

    # 让节点一直运行，以便可以响应 /clear_map 话题的消息和定时器回调
    try:
        rclpy.spin(costmap_subscriber_clearer_node)
    except KeyboardInterrupt:
        pass # 允许通过 Ctrl+C 退出

    costmap_subscriber_clearer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
