#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <thread>
#include <atomic>
#include <serial/serial.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "rclcpp/rclcpp.hpp"

#define BAUDRATE 9600
#define FILTER_SIZE 10

std::atomic_bool imu_thread_running;
std::atomic_bool imu_data_ready;
std::vector<uint8_t> buff;
std::vector<int16_t> acceleration(4, 0);
std::vector<int16_t> angularVelocity(4, 0);
std::vector<int16_t> magnetometer(4, 0);
std::vector<int16_t> angle_degree(4, 0);

// 用于滤波的数据缓冲区
std::vector<std::vector<int16_t>> acc_buffer(FILTER_SIZE, std::vector<int16_t>(4, 0));
std::vector<std::vector<int16_t>> gyro_buffer(FILTER_SIZE, std::vector<int16_t>(4, 0));
std::vector<std::vector<int16_t>> mag_buffer(FILTER_SIZE, std::vector<int16_t>(4, 0));
std::vector<std::vector<int16_t>> angle_buffer(FILTER_SIZE, std::vector<int16_t>(4, 0));
int buffer_index = 0;

class IMUDriverNode : public rclcpp::Node
{
public:
    IMUDriverNode(const std::string& nodeName) : Node(nodeName)
    {
        // 获取串口
        _port_name = this->declare_parameter<std::string>("port_name", "/dev/imu_usb");
        // 发布IMU数据
        // publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/unilidar/imu", 10);
        // 发布磁力计数据
        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
        // IMU驱动线程
        imu_thread_ = std::thread(&IMUDriverNode::imuThread, this, _port_name);
    }

    void joinIMUThread()
    {
        imu_thread_.join();
    }

private:
    void imuThread(const std::string &port_name)
    {
        serial::Serial imu_serial;
        try
        {
            imu_serial.setPort(port_name);
            imu_serial.setBaudrate(BAUDRATE);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
            imu_serial.setTimeout(timeout);
            imu_serial.open();
            RCLCPP_INFO(this->get_logger(), "\033[32mSerial port opened successfully...\033[0m");
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the IMU serial port.");
            return;
        }

        imu_serial.flush();

        while (rclcpp::ok() && imu_thread_running.load())
        {
            if (imu_serial.available())
            {
                uint8_t data;
                imu_serial.read(&data, 1);
                buff.push_back(data);

                if (buff.size() >= 11 && buff[0] == 0x55)
                {
                    std::vector<uint8_t> data_buff(buff.begin(), buff.begin() + 11);
                    std::vector<uint8_t> data(buff.begin() + 2, buff.begin() + 10);
                    bool angle_flag = false;

                    if (data_buff[1] == 0x51)
                    {
                        if (checkSum(data_buff))
                        {
                            acceleration = hexToShort(data);
                            RCLCPP_DEBUG(this->get_logger(), "Acceleration data updated.");
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "0x51 Check failure.");
                        }
                    }
                    else if (data_buff[1] == 0x52)
                    {
                        if (checkSum(data_buff))
                        {
                            angularVelocity = hexToShort(data);
                            RCLCPP_DEBUG(this->get_logger(), "Angular velocity data updated.");
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "0x52 Check failure.");
                        }
                    }
                    else if (data_buff[1] == 0x53)
                    {
                        if (checkSum(data_buff))
                        {
                            angle_degree = hexToShort(data);
                            angle_flag = true;
                            RCLCPP_DEBUG(this->get_logger(), "Angle data updated.");
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "0x53 Check failure.");
                        }
                    }
                    else if (data_buff[1] == 0x54)
                    {
                        if (checkSum(data_buff))
                        {
                            magnetometer = hexToShort(data);
                            RCLCPP_DEBUG(this->get_logger(), "Magnetometer data updated.");
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "0x54 Check failure.");
                        }
                    }

                    buff.clear();

                    if (angle_flag)
                    {
                        imu_data_ready.store(true);
                        RCLCPP_DEBUG(this->get_logger(), "IMU data ready for publishing.");
                    }
                }
                else if (buff[0] != 0x55)
                {
                    buff.clear();
                }
            }

            if (imu_data_ready.load())
            {
                // 更新滤波缓冲区
                updateBuffer(acceleration, acc_buffer);
                updateBuffer(angularVelocity, gyro_buffer);
                updateBuffer(magnetometer, mag_buffer);
                updateBuffer(angle_degree, angle_buffer);
                buffer_index = (buffer_index + 1) % FILTER_SIZE;

                // 计算均值
                std::vector<int16_t> acc_avg = calculateAverage(acc_buffer);
                std::vector<int16_t> gyro_avg = calculateAverage(gyro_buffer);
                std::vector<int16_t> mag_avg = calculateAverage(mag_buffer);
                std::vector<int16_t> angle_avg = calculateAverage(angle_buffer);

                // 发布IMU数据
                sensor_msgs::msg::Imu imu_msg;
                imu_msg.header.stamp = this->now();
                imu_msg.header.frame_id = "base_link";

                imu_msg.linear_acceleration.x = static_cast<double>(acc_avg[0]) / 32768.0 * 16 * 9.8;
                imu_msg.linear_acceleration.y = static_cast<double>(acc_avg[1]) / 32768.0 * 16 * 9.8;
                imu_msg.linear_acceleration.z = static_cast<double>(acc_avg[2]) / 32768.0 * 16 * 9.8;

                imu_msg.angular_velocity.x = static_cast<double>(gyro_avg[0]) / 32768.0 * 2000 * M_PI / 180;
                imu_msg.angular_velocity.y = static_cast<double>(gyro_avg[1]) / 32768.0 * 2000 * M_PI / 180;
                imu_msg.angular_velocity.z = static_cast<double>(gyro_avg[2]) / 32768.0 * 2000 * M_PI / 180;

                double roll = static_cast<double>(angle_avg[0]) / 32768.0 * M_PI;
                double pitch = static_cast<double>(angle_avg[1]) / 32768.0 * M_PI;
                double yaw = static_cast<double>(angle_avg[2]) / 32768.0 * M_PI;
                tf2::Quaternion quat;
                quat.setRPY(roll, pitch, yaw);

                imu_msg.orientation.x = quat.getX();
                imu_msg.orientation.y = quat.getY();
                imu_msg.orientation.z = quat.getZ();
                imu_msg.orientation.w = quat.getW();

                publisher_->publish(imu_msg);
                RCLCPP_INFO(this->get_logger(), "IMU data published.");

                // 发布磁力计数据
                sensor_msgs::msg::MagneticField mag_msg;
                mag_msg.header.stamp = this->now();
                mag_msg.header.frame_id = "base_link";
                mag_msg.magnetic_field.x = static_cast<double>(mag_avg[0]);
                mag_msg.magnetic_field.y = static_cast<double>(mag_avg[1]);
                mag_msg.magnetic_field.z = static_cast<double>(mag_avg[2]);

                mag_publisher_->publish(mag_msg);
                RCLCPP_INFO(this->get_logger(), "Magnetometer data published.");

                imu_data_ready.store(false);
            }
        }
        imu_serial.close();
    }

    bool checkSum(const std::vector<uint8_t> &data_buff)
    {
        uint8_t sum = 0;
        for (size_t i = 0; i < data_buff.size() - 1; i++)
        {
            sum += data_buff[i];
        }

        return sum == data_buff[data_buff.size() - 1];
    }

    std::vector<int16_t> hexToShort(const std::vector<uint8_t> &hex_data)
    {
        std::vector<int16_t> short_data(4, 0);
        for (size_t i = 0; i < hex_data.size(); i += 2)
        {
            int16_t high = static_cast<int16_t>(hex_data[i + 1]);
            short_data[i / 2] = static_cast<int16_t>((high << 8) | hex_data[i]);
        }

        return short_data;
    }

    void updateBuffer(const std::vector<int16_t> &data, std::vector<std::vector<int16_t>> &buffer)
    {
        buffer[buffer_index] = data;
    }

    std::vector<int16_t> calculateAverage(const std::vector<std::vector<int16_t>> &buffer)
    {
        std::vector<int16_t> avg(4, 0);
        for (const auto &data : buffer)
        {
            for (size_t i = 0; i < data.size(); i++)
            {
                avg[i] += data[i];
            }
        }
        for (size_t i = 0; i < avg.size(); i++)
        {
            avg[i] /= FILTER_SIZE;
        }
        return avg;
    }

    std::string _port_name;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    std::thread imu_thread_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto imu_node = std::make_shared<IMUDriverNode>("imu_driver_node");
    imu_thread_running.store(true);

    rclcpp::spin(imu_node);
    imu_thread_running.store(false);

    imu_node->joinIMUThread();

    rclcpp::shutdown();
    return 0;
}

