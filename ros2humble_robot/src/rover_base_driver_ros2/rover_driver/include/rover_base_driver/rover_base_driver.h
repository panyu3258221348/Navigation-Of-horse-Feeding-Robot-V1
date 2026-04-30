
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rover_msgs/msg/chassis_status.hpp>
#include <rover_msgs/msg/battery_status.hpp>
#include <rover_msgs/msg/rc_status.hpp>
#include <rover_msgs/srv/sound_light_control.hpp>

#include "rover_base_driver/data_stream.h"
#include "rover_base_driver/Serial_Async.h"

namespace roverRos{

class roverBaseDriver : public rclcpp::Node
{
private:
    // Parameters
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string serial_port_name_;
    int serial_baud_rate_;
    double linear_scale_;
    double angular_scale_;
    bool pub_odom_tf_;
    double cmd_vel_sub_timeout_;

    std::string imu_frame_id_;
    bool use_imu_;
    bool imu_calibrate_gyro_;
    int imu_cailb_samples_;

    // Subscribers Messages
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

    // Publishers Messages
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<rover_msgs::msg::RCStatus>::SharedPtr rc_state_publisher_;
    rclcpp::Publisher<rover_msgs::msg::BatteryStatus>::SharedPtr battery_state_publisher_;
    rclcpp::Publisher<rover_msgs::msg::ChassisStatus>::SharedPtr chassis_status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Service<rover_msgs::srv::SoundLightControl>::SharedPtr sound_light_server;

    // ROS Internal Variables
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_publish_timer_;
    rclcpp::TimerBase::SharedPtr cmd_vel_sub_timeout_timer_;
    rclcpp::TimerBase::SharedPtr sensor_1hz_publish_timer_;
    rclcpp::TimerBase::SharedPtr sensor_10hz_publish_timer_;
    rclcpp::Time last_odom_vel_time_;
    geometry_msgs::msg::Twist cmd_vel_twis_;
    
    // rover Base Driver Internal Variables
    std::shared_ptr<Serial_Async> serial_port_;
    std::shared_ptr<Data_Stream> data_stream_;
    Data_Format_Liner cmd_vel_;
    float position_x_ = 0.0;
    float position_y_ = 0.0;
    float theta_ = 0.0;

    void GetParameters();
    nav_msgs::msg::Odometry CalculateOdometry(geometry_msgs::msg::Twist &robot_twist);
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twisPtr);
    void SoundLightCtrlCallBack(const rover_msgs::srv::SoundLightControl_Request::SharedPtr req,
                            const rover_msgs::srv::SoundLightControl_Response::SharedPtr res);
    void PublishOdomLoopCallback();
    void publish_imu();
    void PublishSensor1HzLoopCallback();
    void PublishSensor10HzLoopCallback();

    void CmdVelMsgCallbackTimeout()
    {
        cmd_vel_.set(.0, .0, .0);
    }

    /// 运动协方差配置
    void SetOdomCovariance(nav_msgs::msg::Odometry& odom_msg)
    {
        if (odom_msg.twist.twist.linear.x == 0 && odom_msg.twist.twist.angular.z == 0)
        {
            odom_msg.pose.covariance[0]   = 1e-9;
            odom_msg.pose.covariance[7]   = 1e-9;
            odom_msg.pose.covariance[14]  = 1e6;
            odom_msg.pose.covariance[21]  = 1e6;
            odom_msg.pose.covariance[28]  = 1e6;
            odom_msg.pose.covariance[35]  = 1e-9;

            odom_msg.twist.covariance[0]  = 1e-9;
            odom_msg.twist.covariance[7]  = 1e-9;
            odom_msg.twist.covariance[14] = 1e6;
            odom_msg.twist.covariance[21] = 1e6;
            odom_msg.twist.covariance[28] = 1e6;
            odom_msg.twist.covariance[35] = 1e-9;
        }
        else
        {
            odom_msg.pose.covariance[0]   = 1e-3;
            odom_msg.pose.covariance[7]   = 1e-3;
            odom_msg.pose.covariance[14]  = 1e6;
            odom_msg.pose.covariance[21]  = 1e6;
            odom_msg.pose.covariance[28]  = 1e6;
            odom_msg.pose.covariance[35]  = 1e-2;
            
            odom_msg.twist.covariance[0]  = 1e-3;
            odom_msg.twist.covariance[7]  = 1e-3;
            odom_msg.twist.covariance[14] = 1e6;
            odom_msg.twist.covariance[21] = 1e6;
            odom_msg.twist.covariance[28] = 1e6;
            odom_msg.twist.covariance[35] = 1e-2;
        }
    }

    void SetImuCovariance(sensor_msgs::msg::Imu& imu_msg)
    {
        // https://github.com/KristofRobot/razor_imu_9dof/blob/indigo-devel/nodes/imu_node.py
        imu_msg.orientation_covariance[0] = 0.0025;
        imu_msg.orientation_covariance[4] = 0.0025;
        imu_msg.orientation_covariance[8] = 0.0025;

        imu_msg.angular_velocity_covariance[0] = 0.000015;
        imu_msg.angular_velocity_covariance[4] = 0.000015;
        imu_msg.angular_velocity_covariance[8] = 0.000015;

        imu_msg.linear_acceleration_covariance[0] = 0.0001;
        imu_msg.linear_acceleration_covariance[4] = 0.0001;
        imu_msg.linear_acceleration_covariance[8] = 0.0001;
    }

public:
    roverBaseDriver();
  
};

}