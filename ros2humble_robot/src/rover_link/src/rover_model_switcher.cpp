#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/msg/tf_message.hpp>
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
#include "std_msgs/msg/float32.hpp"
#include "rover_interfaces/srv/switch.hpp"
#include <rcutils/cmdline_parser.h>
#include "CRC16.h"

#define MODEL_INIT 16
#define MODEL_HOLD 4
#define MODEL_ACRO 1
#define MODEL_GUIDED 15
#define MODEL_Atuo 10
#define MODEL_Loiter 5
#define MODEL_STEERING 3
#define MODEL_RTL 11
#define MODEL_SMART_RTL 12
#define MODEL_FOLLOW 6
#define MODEL_SIMPLE 7
#define MODEL_MANUAL 0

const uint8_t MODEL_MAN = MODEL_MANUAL;     // 手动模式
const uint8_t MODEL_BUILD_MAP = MODEL_ACRO; // 建图模式
const uint8_t MODEL_NAV = MODEL_GUIDED;     // 导航模式
const uint8_t MODEL_CHARGE = MODEL_RTL;     // 回充模式
const uint8_t MODEL_NONE = MODEL_INIT;      // 默认模式

using std::placeholders::_1;
using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> nodeHandle;

u_int8_t curModel = 0;
u_int8_t apmModel = 0;
rclcpp::Time _Now, _Last_Time; // 时间相关
float sampling_time;           // 采样时间
double _time_now, _time_last;

rclcpp::Service<rover_interfaces::srv::Switch>::SharedPtr switch_model_service;

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr apm_model;
rclcpp::TimerBase::SharedPtr timer;

void gcs_model_cb(const std::shared_ptr<rover_interfaces::srv::Switch::Request> req,
                  std::shared_ptr<rover_interfaces::srv::Switch::Response> res)
{
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "************rover_model_switcher gcs_model_cb***************");

    curModel = req->param1;
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), ">>>>Model: [%d]", (uint8_t)curModel);

    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "sending back response: [%d]", (uint8_t)res->result);
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "***********************************");
}

/**
 * 读取数据
 */
void updateModeStatus()
{

    apmModel = curModel;
}

void Publish_APM_Model()
{
    auto msg_mode = std::make_shared<std_msgs::msg::Float32>();
    msg_mode->data = apmModel;
    apm_model->publish(*msg_mode);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    nodeHandle = rclcpp::Node::make_shared("rover_model_switcher");

    apm_model = nodeHandle->create_publisher<std_msgs::msg::Float32>("apm_model", 10);

    switch_model_service = nodeHandle->create_service<rover_interfaces::srv::Switch>("switch_model", gcs_model_cb);

    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {

        updateModeStatus();
        Publish_APM_Model();

        rclcpp::spin_some(nodeHandle);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
