#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
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
#include <serial/serial.h>
#include <rcutils/cmdline_parser.h>
#include "CRC16.h"
#include "sensor_msgs/msg/imu.hpp"


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
serial::Serial pix_serial;
#define PI 3.1415926f // 圆周率
// 这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define GYROSCOPE_RATIO 0.00026644f // 1/65.5/57.30=0.00026644 陀螺仪原始数据换成弧度单位
// 这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define ACCEl_RATIO 16384.0f // 量程±2g，重力加速度定义为1g等于9.8米每平方秒。

#define SAMPLING_FREQ 20.0f // 采样频率
bool isClientOffline=false; //APP是否掉线
// 协方差
const double odom_pose_covariance[36] =
    {1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] =
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
const double odom_twist_covariance2[36] =
    {1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9};
// 速度/位置结构体
typedef struct __Vel_Pos_Data_
{
    float X;
    float Y;
    float Z;
} Vel_Pos_Data;

Vel_Pos_Data Robot_Pos; // 机器人的位置
Vel_Pos_Data Robot_Vel; // 机器人的速度
std::string usart_port_name;
int serial_baud_rate; // 波特率
float rangfinder_value_1 = 0.0f;
float rangfinder_value_2 = 0.0f;
u_int8_t curModel = 0;
u_int8_t apmModel = 0;
u_int8_t armed = 0;
float Linear_X = 0;
float Argular_Z = 0;
float Linear_X_auto = 0;
float Argular_Z_auto = 0;
float scale_x = 1.0f; // （可选）缩放系数，防止大车型速度过快
float scale_z = 0.3f; // （可选）缩放系数，防止大车型转速过快
bool pub_tf = false;
rclcpp::Time _Now, _Last_Time; // 时间相关
float sampling_time;           // 采样时间
double _time_now, _time_last;

bool isCloseRangeFinder = false;
bool isShutDown = false;
bool isDisArmed = false;

bool isSendShutDown = false;
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_left;
rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_right;
rclcpp::Service<rover_interfaces::srv::Switch>::SharedPtr switch_rangefinder_service;
rclcpp::Service<rover_interfaces::srv::Switch>::SharedPtr switch_model_service;
rclcpp::Service<rover_interfaces::srv::Switch>::SharedPtr switch_disarmed_service;
rclcpp::Client<rover_interfaces::srv::Switch>::SharedPtr client_all_stop;
rclcpp::Service<rover_interfaces::srv::Switch>::SharedPtr client_offline_service;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              pub_imu;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster; // 定义发布器

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr apm_model;
rclcpp::TimerBase::SharedPtr timer;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr arm;
static void aux_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    Linear_X = msg->linear.x;
    Argular_Z = msg->angular.z;
    // RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "aux_cmd_cb:(%f,%f)", Linear_X, Argular_Z);
}
static void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    Linear_X_auto = msg->linear.x;
    Argular_Z_auto = msg->angular.z;
    // RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "cmd_cb:(%f,%f)", Linear_X_auto, Argular_Z_auto);
}
void rangefinder_cb(const std::shared_ptr<rover_interfaces::srv::Switch::Request> req,
                    std::shared_ptr<rover_interfaces::srv::Switch::Response> res)
{
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "************rangefinder_cb***************");

    isCloseRangeFinder = (bool)req->isflag;
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "sending back response: [%d]", (uint8_t)res->result);
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "***********************************");
}
void gcs_model_cb(const std::shared_ptr<rover_interfaces::srv::Switch::Request> req,
                  std::shared_ptr<rover_interfaces::srv::Switch::Response> res)
{
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "************gcs_model_cb***************");

    curModel = req->param1;
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), ">>>>Model: [%d]", (uint8_t)curModel);

    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "sending back response: [%d]", (uint8_t)res->result);
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "***********************************");
}
void client_offline_cb(const std::shared_ptr<rover_interfaces::srv::Switch::Request> req,
                  std::shared_ptr<rover_interfaces::srv::Switch::Response> res)
{
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "************client_offline_cb***************");

    uint8_t flag = req->param1;
    if(flag==1){
       isClientOffline=true;
    }else{
       isClientOffline=false;
    }
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), ">>>>isClientOffline flag: [%d]", (uint8_t)flag);

    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "sending back response: [%d]", (uint8_t)res->result);
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "***********************************");
}


void disArmed_cb(const std::shared_ptr<rover_interfaces::srv::Switch::Request> req,
                 std::shared_ptr<rover_interfaces::srv::Switch::Response> res)
{
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "************disArmed_cb***************");

    isDisArmed = (bool)req->isflag;
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "sending back response: [%d]", (uint8_t)res->result);
    RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "***********************************");
}
float Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |= Data_High << 8;                                       // 获取数据的高8位
    transition_16 |= Data_Low;                                             // 获取数据的低8位
    data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001; //(发送端将数据放大1000倍发送，这里需要将数据单位还原)
    return data_return;
}
/**
 * 检测是否开启关机
 */
void startShutDown()
{
    if (isShutDown == true && isSendShutDown == false) // 关闭全部ROS节点
    {
        isSendShutDown = true;
        auto srv = std::make_shared<rover_interfaces::srv::Switch::Request>();
        srv->isflag = true;

        auto result = client_all_stop->async_send_request(srv);
    }
}
/**
 * 发送数据
 */
void send_cmd(float linear_x, float argular_z)
{
    // 放大1000倍
    int16_t vel_x = linear_x * 1000;
    int16_t vel_z = argular_z * 1000;

    uint16_t len = 13;
    uint8_t cmd_data[len];

    uint16_t cont = 0;
    uint16_t crc;
    cmd_data[cont++] = 0x01;
    cmd_data[cont++] = 0x10; // 多地址写入
    if (isCloseRangeFinder == false)
    {
        cmd_data[cont++] = 0x00; // 开启或关闭超声波（默认开启0x00）
    }
    else
    {
        cmd_data[cont++] = 0x01; // 关闭0x01
    }

    cmd_data[cont++] = 0x04; // 速度寄存器起始地址

    if (isDisArmed == false)
    {
        cmd_data[cont++] = 1;
    }
    else
    {
        cmd_data[cont++] = 2;
    }
    cmd_data[cont++] = curModel;   // 模式
    cmd_data[cont++] = 0x04;       // 负载数据个数
    cmd_data[cont++] = vel_x >> 8; // 寄存器高8位
    cmd_data[cont++] = vel_x;
    cmd_data[cont++] = vel_z >> 8;
    cmd_data[cont++] = vel_z;
    crc = crc16(cmd_data, cont);
    cmd_data[cont++] = crc % 256;
    cmd_data[cont++] = crc / 256;

    try
    {
        pix_serial.write(cmd_data, len); // 向串口发数据
        pix_serial.flush();
    }
    catch (serial::IOException &e)
    {

        RCLCPP_ERROR(rclcpp::get_logger("offboard_serial"), "Unable to send data through serial port");
    }
}

/**
 * 读取数据
 */
void readData()
{
    int length = 200;
    uint8_t buffer_data[length];

    short len = pix_serial.read(buffer_data, pix_serial.available()); // 读串口数据

    uint8_t count = len - 2;
    uint16_t crc = crc16(buffer_data, count);
    if (buffer_data[14] == (crc % 256) && buffer_data[13] == (crc / 256))
    {
        if (len > 0 && len <= 15)
        {
            if (buffer_data[0] == 0x5A && buffer_data[1] == 0x5A) // 帧头
            {
                // apmModel = buffer_data[2]; // APM当前的模式
                apmModel =curModel;
                if (buffer_data[3] == 0x02 || buffer_data[3] == 0x10) // 有效数据长度
                {

                    rangfinder_value_1 = (buffer_data[4] << 8 | buffer_data[5]) * 0.01f;
                    rangfinder_value_2 = (buffer_data[6] << 8 | buffer_data[7]) * 0.01f;
                    Robot_Vel.X = Odom_Trans(buffer_data[8], buffer_data[9]);
                    Robot_Vel.Z = Odom_Trans(buffer_data[10], buffer_data[11]);
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "receive(%f,%f)", Robot_Vel.X, Robot_Vel.Z);

                    // 触发主动关机
                    if (buffer_data[3] == 0x10)
                    {
                        isShutDown = true;
                    }

                    if (buffer_data[3] == 0x02)
                    {
                        isSendShutDown = false;
                        isShutDown = false;
                    }
                    armed = buffer_data[12];
                }
            }
        }
    }
}

void Publish_APM_Model()
{
    auto msg_mode = std::make_shared<std_msgs::msg::Float32>();
    msg_mode->data = apmModel;
    apm_model->publish(*msg_mode);
}

void Publish_APM_Armed()
{
    auto msg_armed = std::make_shared<std_msgs::msg::Float32>();
    msg_armed->data = armed;
    arm->publish(*msg_armed);
}
/**************************************
Date: May 31, 2020
Function: 发布里程计相关信息
***************************************/
void Publish_Odom()
{
    _time_now = nodeHandle->get_clock()->now().seconds();
    sampling_time = _time_now - _time_last;
    Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * sampling_time*0.333; // 计算x方向的位移
    Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * sampling_time*0.333; // 计算y方向的位移，
    Robot_Pos.Z += Robot_Vel.Z * sampling_time * 0.205;                                                // 0.75是用来纠正偏差

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(quaternion);



    auto odom = std::make_unique<nav_msgs::msg::Odometry>();
    odom->header.stamp = nodeHandle->get_clock()->now(); // 当前时间
    odom->header.frame_id = "odom";
    odom->pose.pose.position.x = Robot_Pos.X; // 位置
    odom->pose.pose.position.y = Robot_Pos.Y;
    
    odom->pose.pose.position.z = 0;
    odom->pose.pose.orientation = odom_quat;
    // 设置速度
    odom->child_frame_id = "base_link";
    odom->twist.twist.linear.x = Robot_Vel.X;  // X方向前进速度
    odom->twist.twist.linear.y = Robot_Vel.Y;  // y方向前进速度
    odom->twist.twist.angular.z = Robot_Vel.Z; // 角速度

    //***************************************************************************************************

    // //这个矩阵有两种，分机器人静止和动起来的时候用 这是扩展卡尔曼滤波的,官网提供的2个矩阵，需要配合robot_pose_ekf一起使用，否在沒有意義
    // if (Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0) //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
    // memcpy(&odom->pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
    //     memcpy(&odom->twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    // else //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
    //   memcpy(&odom->pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
    //       memcpy(&odom->twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));

    //****************************************************************************************************
    // first, we'll publish the transform over tf
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = odom->header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = Robot_Pos.X;
    odom_trans.transform.translation.y = Robot_Pos.Y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;
    odom_publisher->publish(std::move(odom)); // 发布这个话题 消息类型是nav_msgs::msg::Odometry
    if (pub_tf == true)
    {

        odom_broadcaster->sendTransform(odom_trans);
    }

    // send the transform

    _time_last = _time_now; // 记录时间
}
/**
 * 发送超声波数据
 */
void publishRangeFinderData()
{
    sensor_msgs::msg::Range rangefinder_left;
    rangefinder_left.header.stamp = nodeHandle->get_clock()->now();
    rangefinder_left.header.frame_id = "rangefinder_left";
    rangefinder_left.radiation_type = 0;
    rangefinder_left.field_of_view = 0.3;
    rangefinder_left.min_range = 0.2;
    rangefinder_left.max_range = 7.2;
    rangefinder_left.range = rangfinder_value_1;
    publisher_left->publish(rangefinder_left);

    sensor_msgs::msg::Range rangefinder_right;
    rangefinder_right.header.stamp = nodeHandle->get_clock()->now();
    rangefinder_right.header.frame_id = "rangefinder_right";
    rangefinder_right.radiation_type = 0;
    rangefinder_right.field_of_view = 0.3;
    rangefinder_right.min_range = 0.2;
    rangefinder_right.max_range = 7.2;
    rangefinder_right.range = rangfinder_value_2;
    publisher_right->publish(rangefinder_right);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    nodeHandle = rclcpp::Node::make_shared("offboard_serial");

    std::string usart_port_name = "/dev/ttyS3";

    char *port_options;
    port_options = rcutils_cli_get_option(argv, argv + argc, "usart_port_name");
    nodeHandle->declare_parameter<bool>("pub_tf", false);
    nodeHandle->get_parameter_or<bool>("pub_tf", pub_tf, false);

    pub_imu = nodeHandle->create_publisher<sensor_msgs::msg::Imu>("/imu", 20);

    if (pub_tf == true)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ">>>>>>>>>>>>>>>>>>>>>>>>>pub_tf==true");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ">>>>>>>>>>>>>>>>>>>>>>>>>pub_tf==false");
    }

    if (nullptr != port_options)
    {
        usart_port_name = std::string(port_options);
    }

    serial_baud_rate = 115200;

    isShutDown = false;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "init********Serial name :%s,  baudrate:%d", usart_port_name.c_str(), serial_baud_rate);

    auto aux_cmd_sub = nodeHandle->create_subscription<geometry_msgs ::msg::Twist>("cmd_vel/mobile_app", rclcpp::SensorDataQoS(), aux_cmd_cb);
    auto cmd_sub = nodeHandle->create_subscription<geometry_msgs ::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), cmd_cb);

    publisher_left = nodeHandle->create_publisher<sensor_msgs::msg::Range>("rangefinder/left", 1);
    publisher_right = nodeHandle->create_publisher<sensor_msgs::msg::Range>("rangefinder/rigth", 1);
    apm_model = nodeHandle->create_publisher<std_msgs::msg::Float32>("apm_model", 10);
    
    arm = nodeHandle->create_publisher<std_msgs::msg::Float32>("apm_disarm", 10);


    // 里程计数据发布
    odom_publisher = nodeHandle->create_publisher<nav_msgs::msg::Odometry>("/wheel_odom", 10);

    switch_rangefinder_service = nodeHandle->create_service<rover_interfaces::srv::Switch>("switch_rangefinder", rangefinder_cb);
    switch_model_service = nodeHandle->create_service<rover_interfaces::srv::Switch>("switch_model", gcs_model_cb);
    client_offline_service= nodeHandle->create_service<rover_interfaces::srv::Switch>("client_offline", client_offline_cb);
    switch_disarmed_service = nodeHandle->create_service<rover_interfaces::srv::Switch>("switch_disarmed", disArmed_cb);
    client_all_stop = nodeHandle->create_client<rover_interfaces::srv::Switch>("cmd_all_stop");

    odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(nodeHandle);
    try
    {
        pix_serial.setPort(usart_port_name);      // 选择哪个口，如果选择的口没有接串口外设初始化会失败
        pix_serial.setBaudrate(serial_baud_rate); // 设置波特率

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "********Serial name :%s,  baudrate:%d", usart_port_name.c_str(), serial_baud_rate);

        serial::Timeout time = serial::Timeout::simpleTimeout(2000); // 超时等待
        pix_serial.setTimeout(time);
        pix_serial.open(); // 串口开启
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "********pix_serial.open");
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), " can not open Serial port,Please check the serial port cable! ");
        return EXIT_SUCCESS;
    }


    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        if (curModel == MODEL_MAN || curModel == MODEL_BUILD_MAP)
        {
            if(isClientOffline==true){
                 send_cmd(0, 0);   //APP异常掉线

            }else{
                send_cmd(Linear_X, Argular_Z);  //APP虚拟摇杆控制
            }
        }
        else
        {
            send_cmd(Linear_X_auto, Argular_Z_auto);  //自动导航控制
        }
        Publish_Odom();
        readData();
        publishRangeFinderData();
        Publish_APM_Model();
        Publish_APM_Armed();
        startShutDown();

        rclcpp::spin_some(nodeHandle);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
