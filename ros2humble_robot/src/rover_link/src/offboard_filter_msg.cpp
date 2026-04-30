#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <zconf.h>
#include <ifaddrs.h>
#include <memory>

#include "mavros_msgs/srv/command_bool.hpp"
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>

#include "rover_interfaces/srv/switch.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> nodeHandle;

rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr aux_cmd_publisher;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr reverse_cmd_publisher;

// 协方差
const double odom_pose_covariance[36] =
    {1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = // 静止时
    {1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] =
    {1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = // 静止时
    {1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9};

void aux_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::TwistStamped twistStampted;
    twistStampted.twist.linear = msg->linear;
    twistStampted.twist.angular = msg->angular;

    aux_cmd_publisher->publish(twistStampted);
}

void reverse_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::Twist twist;
    twist.linear = msg->linear;
    twist.angular = msg->angular;

    twist.linear.x = -msg->linear.x;
    twist.angular.z = -msg->angular.z;
    reverse_cmd_publisher->publish(twist);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    nodeHandle = rclcpp::Node::make_shared("offboard_filter_msg");
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr aux_cmd_sub = nodeHandle->create_subscription<geometry_msgs ::msg::Twist>("yocs_cmd_vel_mux/output/cmd_vel", 10, aux_cmd_cb);
    aux_cmd_publisher = nodeHandle->create_publisher<geometry_msgs::msg::TwistStamped>("yocs_cmd_vel_mux/output/cmd_vel_stamped", 10);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reverse_cmd_sub = nodeHandle->create_subscription<geometry_msgs ::msg::Twist>("cmd_vel", 10, reverse_cmd_cb);
    reverse_cmd_publisher = nodeHandle->create_publisher<geometry_msgs::msg::Twist>("cmd_vel/reverse", 10);
    rclcpp::WallRate loop_rate(10);
    while (rclcpp::ok())
    {

        rclcpp::spin_some(nodeHandle);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
