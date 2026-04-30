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

using std::placeholders::_1;
using namespace std::chrono_literals;

const uint8_t MODEL_MAN = 0;       // 手动模式
const uint8_t MODEL_BUILD_MAP = 1; // 建图模式
const uint8_t MODEL_NAV = 15;       // 导航模式
const uint8_t MODEL_CHARGE = 11;    // 回充模式
const uint8_t MODEL_NONE = 16;      // 默认模式

// 扫描待发缓冲池的频率，可以通过增大该值来加快发送的响应速度
const uint8_t HZ = 5;
const uint8_t PAYLOAD_SIZE = 253;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> nodeHandle = rclcpp::Node::make_shared("offboard_nav_manager");
    rclcpp::Client<rover_interfaces::srv::Switch>::SharedPtr client =
        nodeHandle->create_client<rover_interfaces::srv::Switch>("switch_model");

    std::this_thread::sleep_for(std::chrono::seconds(10)); // 延时10s，等待RTAB启动完成

    auto srv = std::make_shared<rover_interfaces::srv::Switch::Request>();
    srv->param1 = MODEL_NAV; // 导航模式

    auto result = client->async_send_request(srv);

    rclcpp::spin(nodeHandle);
    rclcpp::shutdown();
    return 0;
}
