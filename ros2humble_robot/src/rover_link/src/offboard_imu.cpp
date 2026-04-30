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


using std::placeholders::_1;
using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> nodeHandle;
serial::Serial imu_serial;
#define PI 3.1415926f // 圆周率
// 这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define GYROSCOPE_RATIO 0.00026644f // 1/65.5/57.30=0.00026644 陀螺仪原始数据换成弧度单位
// 这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define ACCEl_RATIO 16384.0f // 量程±2g，重力加速度定义为1g等于9.8米每平方秒。

#define SAMPLING_FREQ 20.0f // 采样频率

int serial_baud_rate; // 波特率
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    pub_imu;

rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    pub_temp;
rclcpp::TimerBase::SharedPtr timer;

void timer_callback(){

    int length = 200;
    uint8_t buffer_data[length];

    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    uint8_t tempture;
    
    short len = imu_serial.read(buffer_data, imu_serial.available()); // 读串口数据
    if(len>0){


      uint8_t count = len - 2;
      uint16_t crc = crc16(buffer_data, count);
      if (buffer_data[28] == (crc % 256) && buffer_data[27] == (crc / 256))
      {
        if (len > 0 && len <= 29)
        {
            if (buffer_data[0] == 0x5A && buffer_data[1] == 0x5A) // 帧头
            {

                accel_x = ((buffer_data[2] << 24) | (buffer_data[3] << 16) | (buffer_data[4] << 8) | buffer_data[5])*0.001;
                accel_y = ((buffer_data[6] << 24) | (buffer_data[7] << 16) | (buffer_data[8] << 8) | buffer_data[9])*0.001;
                accel_z = ((buffer_data[10] << 24) | (buffer_data[11] << 16) | (buffer_data[12] << 8) | buffer_data[13])*0.001;

                gyro_x = ((buffer_data[14] << 24) | (buffer_data[15] << 16) | (buffer_data[16] << 8) | buffer_data[17])*0.001;
                gyro_y = ((buffer_data[18] << 24) | (buffer_data[19] << 16) | (buffer_data[20] << 8) | buffer_data[21])*0.001;
                gyro_z = ((buffer_data[22] << 24) | (buffer_data[23] << 16) | (buffer_data[24] << 8) | buffer_data[25])*0.001;

                tempture = buffer_data[26];

            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Accel: (%f,%f,%f)   Gyro: (%f,%f,%f)   tempture=%d",accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,tempture);

            rclcpp::Time now = nodeHandle->get_clock()->now();
            sensor_msgs::msg::Imu imu_data;
            imu_data.header.stamp = now;
            imu_data.header.frame_id = "gyro_link";

            imu_data.linear_acceleration.x = accel_x;
            imu_data.linear_acceleration.y = accel_y;
            imu_data.linear_acceleration.z = accel_z;
            
            imu_data.angular_velocity.x = gyro_x ;
            imu_data.angular_velocity.y = gyro_y ;
            imu_data.angular_velocity.z = gyro_z ;

            tf2::Quaternion curr_quater;
            curr_quater.setRPY(gyro_x, gyro_y, gyro_z); //zyx

            imu_data.orientation.x = curr_quater.x();
            imu_data.orientation.y = curr_quater.y();
            imu_data.orientation.z = curr_quater.z();
            imu_data.orientation.w = curr_quater.w();


            pub_imu->publish(imu_data);

            std_msgs::msg::Float32 temp;
            temp.data=tempture;
            pub_temp->publish(temp);

              
            }
        }
      }
    }

}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    nodeHandle = rclcpp::Node::make_shared("offboard_imu");

    std::string imu_port_name = "/dev/ttyS6";

    char *port_options;
    port_options = rcutils_cli_get_option(argv, argv + argc, "usart_port_name");
    pub_imu = nodeHandle->create_publisher<sensor_msgs::msg::Imu>("/imu", 20);
    pub_temp = nodeHandle->create_publisher<std_msgs::msg::Float32>("temperature", 10);

   

    if (nullptr != port_options)
    {
        imu_port_name = std::string(port_options);
    }

    serial_baud_rate = 115200;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "init******** IMU Serial name :%s,  baudrate:%d", imu_port_name.c_str(), serial_baud_rate);
    
    timer=nodeHandle->create_wall_timer(std::chrono::milliseconds(10),&timer_callback);//100Hz



    try
    {
        imu_serial.setPort(imu_port_name);      // 选择哪个口，如果选择的口没有接串口外设初始化会失败
        imu_serial.setBaudrate(serial_baud_rate); // 设置波特率

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "********IMU Serial name :%s,  baudrate:%d", imu_port_name.c_str(), serial_baud_rate);

        serial::Timeout time = serial::Timeout::simpleTimeout(2000); // 超时等待
        imu_serial.setTimeout(time);
        imu_serial.open(); // 串口开启
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "********imu_serial.open");
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), " can not open imu Serial port,Please check the IMU serial port cable! ");
        return EXIT_SUCCESS;
    }

    rclcpp::spin(nodeHandle);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
