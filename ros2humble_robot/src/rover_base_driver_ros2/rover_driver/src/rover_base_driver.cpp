
#include "rover_base_driver.h"
#include "calibrate_gyro.h"
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/quaternion.hpp>
#ifdef TF2_CPP_HEADERS
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

using std::placeholders::_1;

using namespace roverRos;

#define ROS2_READ_PARAM(TYPE, NAME, VAR, VALUE) \
    VAR = VALUE;                                \
    this->declare_parameter<TYPE>(NAME, VAR);   \
    this->get_parameter(NAME, VAR);

void roverBaseDriver::GetParameters()
{
    ROS2_READ_PARAM(std::string, "port_name", (serial_port_name_), "/dev/rover");
    ROS2_READ_PARAM(int, "port_baud", (serial_baud_rate_), 230400);
    ROS2_READ_PARAM(std::string, "odom_frame_id", (odom_frame_id_), "odom");
    ROS2_READ_PARAM(std::string, "base_frame_id", (base_frame_id_), "base_link");
    ROS2_READ_PARAM(bool, "pub_odom_tf", (pub_odom_tf_), false);
    ROS2_READ_PARAM(double, "cmd_vel_sub_timeout", (cmd_vel_sub_timeout_), 2.0);

    ROS2_READ_PARAM(double, "linear_scale", (linear_scale_), 1.0);
    ROS2_READ_PARAM(double, "angular_scale", (angular_scale_), 1.0);

    ROS2_READ_PARAM(std::string, "imu_frame_id", (imu_frame_id_), "imu_link");
    ROS2_READ_PARAM(bool, "publish_imu", (use_imu_), false);
    ROS2_READ_PARAM(bool, "imu_calibrate_gyro", (imu_calibrate_gyro_), true);
    ROS2_READ_PARAM(int, "imu_cailb_samples", (imu_cailb_samples_), 300);
}

nav_msgs::msg::Odometry roverBaseDriver::CalculateOdometry(geometry_msgs::msg::Twist &robot_twist)
{
    auto current_time = this->now();

    float linear_velocity_x = robot_twist.linear.x * linear_scale_;
    float linear_velocity_y = robot_twist.linear.y * linear_scale_;
    float angular_velocity_z = robot_twist.angular.z * angular_scale_;

    double vel_dt_ = (current_time - last_odom_vel_time_).seconds();
    last_odom_vel_time_ = current_time;

    double delta_x = (linear_velocity_x * cos(theta_) - linear_velocity_y * sin(theta_)) * vel_dt_; // m
    double delta_y = (linear_velocity_x * sin(theta_) + linear_velocity_y * cos(theta_)) * vel_dt_; // m
    double delta_theta = angular_velocity_z * vel_dt_;                                              // radians

    position_x_ += delta_x;
    position_y_ += delta_y;
    theta_ += delta_theta;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation = tf2::toMsg(odom_quat);

    odom_msg.twist.twist.linear.x = robot_twist.linear.x;
    odom_msg.twist.twist.linear.y = robot_twist.linear.y;
    odom_msg.twist.twist.angular.z = robot_twist.angular.z;

    SetOdomCovariance(odom_msg);

    return odom_msg;
}

void roverBaseDriver::PublishSensor1HzLoopCallback()
{
    if (data_stream_->get_Message(MSG_ID_GET_VOLTAGE))
    {
        rover_msgs::msg::BatteryStatus batStatus;
        auto rxData_battery = data_stream_->get_data_battery();

        batStatus.voltage = rxData_battery.voltage / 100.0;
        batStatus.current = rxData_battery.current / 100.0;
        batStatus.percentage = rxData_battery.percentage;
        batStatus.temperature = rxData_battery.temperature / 10.0;
        batStatus.bms_connect = false;

        battery_state_publisher_->publish(batStatus);
    }
    else
    {
        RCLCPP_WARN_STREAM(this->get_logger(), "Get VOLTAGE Data Time Out!");
    }
}

void roverBaseDriver::PublishSensor10HzLoopCallback()
{
    if (data_stream_->get_Message(MSG_ID_GET_RC))
    {
        auto rxData_rc = data_stream_->get_data_rc();
        rover_msgs::msg::RCStatus rc_msg;

        rc_msg.header.stamp = this->now();
        rc_msg.connect = rxData_rc.connect;
        rc_msg.channel[0] = rxData_rc.ch1;
        rc_msg.channel[1] = rxData_rc.ch2;
        rc_msg.channel[2] = rxData_rc.ch3;
        rc_msg.channel[3] = rxData_rc.ch4;
        rc_msg.channel[4] = rxData_rc.ch5;
        rc_msg.channel[5] = rxData_rc.ch6;
        rc_msg.channel[6] = rxData_rc.ch7;
        rc_msg.channel[7] = rxData_rc.ch8;
        rc_msg.channel[8] = rxData_rc.ch9;
        rc_msg.channel[9] = rxData_rc.ch10;

        rc_state_publisher_->publish(rc_msg);
    }
    // else
    //     RCLCPP_WARN_STREAM(this->get_logger(), "Get Remote Control Data Time Out!");

    if (chassis_status_publisher_)
    {
        if (data_stream_->get_Message(MSG_ID_GET_STATUS))
        {
            auto rxData_status = data_stream_->get_data_status();
            rover_msgs::msg::ChassisStatus chassisStatus;
            chassisStatus.header.stamp = this->now();
            chassisStatus.system_status = rxData_status.system_status;
            chassisStatus.anti_collision_status = rxData_status.anti_collision_status;
            chassisStatus.anti_drop_status = rxData_status.anti_drop_status;
            chassisStatus.charging_pile = rxData_status.charging_pile;
            chassisStatus.temperature = rxData_status.temperature / 10.0;
            chassisStatus.humidity = rxData_status.humidity / 10.0;
            chassisStatus.barometer = rxData_status.barometer;
            chassis_status_publisher_->publish(chassisStatus);
        }
    }
}

void roverBaseDriver::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twisPtr)
{
    cmd_vel_.set(twisPtr->linear.x, twisPtr->linear.y, twisPtr->angular.z);
    cmd_vel_sub_timeout_timer_->reset();
}

void roverBaseDriver::publish_imu()
{
    if (data_stream_->get_Message(MSG_ID_GET_IMU))
    {
        auto imu_data = data_stream_->get_data_imu();
        sensor_msgs::msg::Imu imu_msg;
        if (imu_calibrate_gyro_)
        {
            static calibrate_gyro calibGyro(imu_cailb_samples_);
            bool isCailb = calibGyro.calib(this, imu_data.angx, imu_data.angy, imu_data.angz);
            if (isCailb == false)
            {
                return;
            }

            imu_msg.angular_velocity.x = calibGyro.calib_x;
            imu_msg.angular_velocity.y = calibGyro.calib_y;
            imu_msg.angular_velocity.z = calibGyro.calib_z;
        }
        else
        {
            imu_msg.angular_velocity.x = imu_data.angx;
            imu_msg.angular_velocity.y = imu_data.angy;
            imu_msg.angular_velocity.z = imu_data.angz;
        }

        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = imu_frame_id_;
        imu_msg.linear_acceleration.x = imu_data.accx * 9.80665; // 加速度应以 m/s^2（原单位 g ）
        imu_msg.linear_acceleration.y = imu_data.accy * 9.80665;
        imu_msg.linear_acceleration.z = imu_data.accz * 9.80665;

        tf2::Quaternion imu_quat;
        imu_quat.setRPY(imu_data.roll, imu_data.pitch, imu_data.yaw);
        imu_msg.orientation = tf2::toMsg(imu_quat);
        SetImuCovariance(imu_msg);

        imu_publisher_->publish(imu_msg);
    }
}

void roverBaseDriver::PublishOdomLoopCallback()
{
    if (serial_port_->isOpen() == false)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Serial closes unexpectedly!");
        rclcpp::shutdown();
        return;
    }

    static Data_Format_Liner linertx;
    linertx.EndianSwapSet(&cmd_vel_);
    data_stream_->update_liner_speed(linertx);

    if (data_stream_->get_Message(MSG_ID_GET_VELOCITY) == true)
    {
        Data_Format_Liner liner_rx_ = data_stream_->get_data_liner();

        geometry_msgs::msg::Twist robot_twist;
        robot_twist.linear.x = liner_rx_.v_liner_x;
        robot_twist.linear.y = liner_rx_.v_liner_y;
        robot_twist.linear.z = 0;
        robot_twist.angular.x = 0;
        robot_twist.angular.y = 0;
        robot_twist.angular.z = liner_rx_.v_angular_z;

        auto odom_msg = CalculateOdometry(robot_twist);
        if (pub_odom_tf_)
        {
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header = odom_msg.header;
            odom_tf.child_frame_id = odom_msg.child_frame_id;

            // robot's position in x,y, and z
            odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
            odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
            odom_tf.transform.translation.z = 0.0;

            // robot's heading in quaternion
            odom_tf.transform.rotation.x = odom_msg.pose.pose.orientation.x;
            odom_tf.transform.rotation.y = odom_msg.pose.pose.orientation.y;
            odom_tf.transform.rotation.z = odom_msg.pose.pose.orientation.z;
            odom_tf.transform.rotation.w = odom_msg.pose.pose.orientation.w;

            odom_broadcaster_->sendTransform(odom_tf);
            odom_publisher_->publish(odom_msg);

            // RCLCPP_WARN_STREAM(this->get_logger(), "odom_broadcaster_->sendTransform");
        }
    }
    else
        RCLCPP_WARN_STREAM(this->get_logger(), "Get VELOCITY Data Time Out!");

    if (imu_publisher_)
    {
        publish_imu();
    }
}

void roverBaseDriver::SoundLightCtrlCallBack(const rover_msgs::srv::SoundLightControl_Request::SharedPtr req,
                                             const rover_msgs::srv::SoundLightControl_Response::SharedPtr res)
{
    Data_Format_Sound_Light sl;
    sl.light_status = req->light_status;
    sl.light_red_color = req->light_rgb_color[0];
    sl.light_green_color = req->light_rgb_color[1];
    sl.light_blue_color = req->light_rgb_color[2];
    sl.beep_status = req->beep_status;
    sl.searchlight_status = req->headlight_status;

    data_stream_->SetSoundLight(sl);
    res->success = true;
}

roverBaseDriver::roverBaseDriver() : Node("rover_base_driver")
{
    GetParameters();

    serial_port_ = std::make_shared<Serial_Async>();
    data_stream_ = std::make_shared<Data_Stream>(serial_port_.get());

    if (serial_port_->init(serial_port_name_, serial_baud_rate_))
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Main board Serial Port open success, com_port_name= " << serial_port_name_);
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Main board Serial Port open failed... com_port_name= " << serial_port_name_);
        return;
    }

    if (data_stream_->version_detection())
    {
        auto version = data_stream_->get_data_version();
        RCLCPP_INFO_STREAM(this->get_logger(), "The version matches successfully, current version: [" << (int)version.protoVer << "]");
        RCLCPP_INFO_STREAM(this->get_logger(), "GET Equipment Identity: " << version.equipmentIdentity);
    }
    else
    {
        auto version = data_stream_->get_data_version();
        RCLCPP_ERROR_STREAM(this->get_logger(), "The driver version does not match,  Main control board driver version:["
                                                    << (int)version.protoVer << "] Current driver version:[" << LA_PROTO_VER_0330 << "]");
        return;
    }

    // setup subscribers
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("~/cmd_vel", 5, std::bind(&roverBaseDriver::CmdVelCallback, this, _1));

    // setup publishers
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SensorDataQoS());
    rc_state_publisher_ = this->create_publisher<rover_msgs::msg::RCStatus>("~/rc_state", 10);
    battery_state_publisher_ = this->create_publisher<rover_msgs::msg::BatteryStatus>("~/battery_state", 5);

    if (use_imu_)
    {
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("~/onboard_imu", rclcpp::SensorDataQoS());
    }

    if (data_stream_->LightControlAvailable())
    {
        sound_light_server = this->create_service<rover_msgs::srv::SoundLightControl>("~/sound_light_ctrl", std::bind(&roverBaseDriver::SoundLightCtrlCallBack, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (data_stream_->StatusAvailable())
    {
        chassis_status_publisher_ = this->create_publisher<rover_msgs::msg::ChassisStatus>("~/chasssis_status", 1);
    }

    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_odom_vel_time_ = this->now();

    odom_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&roverBaseDriver::PublishOdomLoopCallback, this));

    cmd_vel_sub_timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((uint16_t)(1000 * cmd_vel_sub_timeout_)),
        std::bind(&roverBaseDriver::CmdVelMsgCallbackTimeout, this));

    sensor_1hz_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&roverBaseDriver::PublishSensor1HzLoopCallback, this));

    sensor_10hz_publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&roverBaseDriver::PublishSensor10HzLoopCallback, this));
}
