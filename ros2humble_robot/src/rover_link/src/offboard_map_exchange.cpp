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

#include "sensor_msgs/msg/compressed_image.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

const uint8_t MODEL_MAN = 0;       // 手动模式
const uint8_t MODEL_BUILD_MAP = 1; // 建图模式
const uint8_t MODEL_NAV = 15;       // 导航模式
const uint8_t MODEL_CHARGE = 11;    // 回充模式
const uint8_t MODEL_NONE = 16;      // 默认模式
bool isAgain;
struct SendData
{

    uint8_t *data_0; // 通道0的数据内容
    uint8_t *data_1;
    uint8_t *data_2;
    uint8_t *data_3;
    uint8_t *data_4; // 通道0的数据内容
    uint8_t *data_5;
    uint8_t *data_6;
    uint8_t *data_7;
    uint8_t *data_8; // 通道0的数据内容
    uint8_t *data_9;

    uint32_t size_0; // 通道0的数据大小
    uint32_t size_1;
    uint32_t size_2;
    uint32_t size_3;
    uint32_t size_4; // 通道0的数据大小
    uint32_t size_5;
    uint32_t size_6;
    uint32_t size_7;
    uint32_t size_8; // 通道0的数据大小
    uint32_t size_9;

    uint16_t width;    // 该数据对象的宽
    uint16_t height;   // 该数据对象的高
    double resolution; // 地图分辨率
    double origin_x;   // 起点坐标X
    double origin_y;   // 起点坐标Y
    bool isflag;
};

// 缓冲池
std::vector<SendData> send_data_buffer;
// 最终要发送的数据对象
SendData send_data_final;
const uint8_t HZ = 5;
std::shared_ptr<rclcpp::Node> nodeHandle;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_first_frame_publisher;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_ex_publisher;

rclcpp::Client<rover_interfaces::srv::Switch>::SharedPtr client;

void compressedImage_cb(const sensor_msgs::msg::CompressedImage::SharedPtr compressedImage)
{

    nav_msgs::msg::OccupancyGrid image_ex;
    image_ex.header = compressedImage->header;
    signed char *dataBytes_0 = new signed char[compressedImage->data.size()];
    for (u_int32_t i = 0; i < compressedImage->data.size(); i++)
    {
        dataBytes_0[i] = compressedImage->data[i];
        image_ex.data.push_back(dataBytes_0[i]);
    }
    map_ex_publisher->publish(image_ex);
}
void OccupancyGrid_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

    // if (isAgain == true)
    // {
        // 转换数据
        SendData send_data;
        send_data.isflag = true;

        send_data.width = msg->info.width;
        send_data.height = msg->info.height;
        send_data.resolution = msg->info.resolution;
        send_data.origin_x = msg->info.origin.position.x;
        send_data.origin_y = msg->info.origin.position.y;
        send_data_buffer.emplace_back(send_data); // 加入缓存容器，等待Mavlink发送

        nav_msgs::msg::OccupancyGrid first_frame;
        first_frame.header = msg->header;
        first_frame.info = msg->info;
        first_frame.data = msg->data;
        grid_map_first_frame_publisher->publish(first_frame);
    // }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    nodeHandle = rclcpp::Node::make_shared("offboard_map_exchange");
    isAgain = true;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr state_sub = nodeHandle->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, OccupancyGrid_cb);
    client = nodeHandle->create_client<rover_interfaces::srv::Switch>("switch_model");

    grid_map_first_frame_publisher = nodeHandle->create_publisher<nav_msgs::msg::OccupancyGrid>("grid_map_first_frame", 1);
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressedImage_sub = nodeHandle->create_subscription<sensor_msgs::msg::CompressedImage>("map_image/full/compressed", 1, compressedImage_cb);
    map_ex_publisher = nodeHandle->create_publisher<nav_msgs::msg::OccupancyGrid>("map_image_compressed", 1);
    rclcpp::WallRate loop_rate(5);
    while (rclcpp::ok())
    {
        if (send_data_buffer.size() > 0 && isAgain == true)
        {
            isAgain = false;
        RCLCPP_ERROR(rclcpp::get_logger("offboard_map_exchange"), ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>offboard_map_exchange");

            auto srv = std::make_shared<rover_interfaces::srv::Switch::Request>();
            srv->param1 = MODEL_BUILD_MAP; // 建图模式

            auto result = client->async_send_request(srv);
        }

        rclcpp::spin_some(nodeHandle);
        loop_rate.sleep(); // 持续时间（毫秒） 200ms，即5HZ
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
