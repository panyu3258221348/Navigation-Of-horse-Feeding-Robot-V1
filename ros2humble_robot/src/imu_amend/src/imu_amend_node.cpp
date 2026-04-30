#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <deque>
#include <algorithm>
#include <numeric>
#include <cmath>

// 全局变量存储接收到的速度
double g_current_linear_velocity = 0.0;
double g_current_angular_velocity = 0.0;
double g_current_angular = 0.0;

class StraightController : public rclcpp::Node {
public:
  StraightController() : Node("imu_straight_controller"),
                        first_imu_received_(false),
                        target_speed_(0.3),
                        max_steering_(1.5),
                        target_angle_(0.0) {
    // 参数声明和初始化
    declare_parameters();
    initialize_parameters();
    
    // 初始化坐标系变换矩阵
    initialize_rotation_matrix();
    
    // 订阅和发布
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&StraightController::imu_callback, this, std::placeholders::_1));
    
    // 新增cmd_vel订阅者
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&StraightController::cmd_vel_callback, this, std::placeholders::_1));
    
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    
    // PID参数初始化
    init_pid_controllers();
    
    // 参数变更回调
    param_handler_ = add_on_set_parameters_callback(
      std::bind(&StraightController::parameters_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Cascade PID controller initialized");
  }
 
  bool is_first_call = true;
private:
  struct PIDParams {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double integral = 0.0;
    double last_error = 0.0;
    rclcpp::Time last_time;
    double integral_limit = 0.0;
  };

  // 新增cmd_vel回调函数
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // 更新全局速度变量
    g_current_linear_velocity = msg->linear.x;
    g_current_angular_velocity = msg->angular.z;

   
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Received velocities - linear: %.2f m/s, angular: %.2f rad/s",
      g_current_linear_velocity, g_current_angular_velocity);
  }

  void declare_parameters() {
    // 控制参数
    declare_parameter<double>("target_speed", 0.3);
    declare_parameter<double>("max_steering", 1.5);
    declare_parameter<double>("target_angle", 0.0);
    
    // PID参数
    declare_parameter<double>("angle_kp", 1.2);
    declare_parameter<double>("angle_ki", 0.01);
    declare_parameter<double>("angle_kd", 0.2);
    declare_parameter<double>("angle_integral_limit", 0.5);
    
    declare_parameter<double>("angular_vel_kp", 0.8);
    declare_parameter<double>("angular_vel_ki", 0.0);
    declare_parameter<double>("angular_vel_kd", 0.1);
    declare_parameter<double>("angular_vel_integral_limit", 1.0);
    
    // 滤波器参数
    declare_parameter<int>("angle_filter_window", 5);
    declare_parameter<int>("angular_vel_filter_window", 5);
    
    // 坐标系参数
    declare_parameter<std::vector<double>>("rotation_matrix", 
      {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    
    // 话题参数
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  }

  void initialize_parameters() {
    // 缓存高频访问参数
    target_speed_ = get_parameter("target_speed").as_double();
    max_steering_ = get_parameter("max_steering").as_double();
    target_angle_ = get_parameter("target_angle").as_double();
    imu_topic_ = get_parameter("imu_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    
    // 初始化滤波器窗口
    angle_filter_window_ = get_parameter("angle_filter_window").as_int();
    angular_vel_filter_window_ = get_parameter("angular_vel_filter_window").as_int();
  }

  void initialize_rotation_matrix() {
    auto rot_params = get_parameter("rotation_matrix").as_double_array();
    if (rot_params.size() != 9) {
      RCLCPP_ERROR(get_logger(), "Invalid rotation matrix size! Using identity matrix.");
      rotation_matrix_ = Eigen::Matrix3d::Identity();
    } else {
      rotation_matrix_ = Eigen::Map<Eigen::Matrix3d>(rot_params.data()).transpose();
    }
  }

  void init_pid_controllers() {
    // 角度环PID
    angle_pid_.kp = get_parameter("angle_kp").as_double();
    angle_pid_.ki = get_parameter("angle_ki").as_double();
    angle_pid_.kd = get_parameter("angle_kd").as_double();
    angle_pid_.integral_limit = get_parameter("angle_integral_limit").as_double();
    angle_pid_.last_time = now();
    
    // 角速度环PID
    angular_vel_pid_.kp = get_parameter("angular_vel_kp").as_double();
    angular_vel_pid_.ki = get_parameter("angular_vel_ki").as_double();
    angular_vel_pid_.kd = get_parameter("angular_vel_kd").as_double();
    angular_vel_pid_.integral_limit = get_parameter("angular_vel_integral_limit").as_double();
    angular_vel_pid_.last_time = now();
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
   // RCLCPP_INFO(get_logger(), "Raw angular velocity: x=%.3f, y=%.3f, z=%.3f",
                // msg->angular_velocity.x,
                // msg->angular_velocity.y,
                // msg->angular_velocity.z);
    // 1. 坐标变换
    float angular_vel_body= msg->angular_velocity.z;
  
    // 2. 角度计算
    double current_yaw = get_yaw_from_imu(msg);
    
    // 3. 角度滤波
    double filtered_yaw = current_yaw;
    
   static double g_current_angular_velocity_last;
   
   static auto t_last=now();
   
   if(fabs(g_current_angular_velocity)>0.055)
   {
    two_imu_received_=true;
    // first_imu_received_ = false;
    if(fabs(target_angle_)!=3.141593)
       target_angle_=target_angle_+(g_current_angular_velocity/1.6)*((now()-t_last).seconds());   
    else
      target_angle_=target_angle_+(g_current_angular_velocity/0.3)*((now()-t_last).seconds());  
    if(target_angle_>3.14)
    {
      target_angle_=3.141593;
    }
    if(target_angle_<-3.14)
    {
      target_angle_=-3.141593;
    }
    
    // target_angle_=((float)(((int)(target_angle_*1000))%3141))/1000.0;

    RCLCPP_INFO(get_logger(), "Initial target angle set to: %.6f rad", target_angle_);
    t_last=now();
    // target_angle_=g_current_angular;
    
    // g_current_angular_velocity_last=g_current_angular_velocity;
   }
   else
   {
     t_last=now();
    if(two_imu_received_)
    {
       two_imu_received_=false;
       first_imu_received_ = false;
    }
   }
    // 4. 初始角度锁定
    if (!first_imu_received_) {
      target_angle_ = filtered_yaw;
      first_imu_received_ = true;
      RCLCPP_INFO(get_logger(), "Initial target angle set to: %.3f rad", target_angle_);
    }
    
    RCLCPP_INFO(get_logger(), "filtered_yaw: %.3f rad\n", filtered_yaw);
    RCLCPP_INFO(get_logger(), "target_angle_: %.3f rad\n", target_angle_);
    // RCLCPP_INFO(get_logger(), "angular_vel_body: %.3f rad/s\n", angular_vel_body);
    
    // 5. 外环PID计算（角度环）
    double target_angular_vel = angle_pid_update(filtered_yaw);
    RCLCPP_INFO(get_logger(), "target_angular_vel: %.3f rad\n", target_angular_vel);
    
    // 7. 内环PID计算（角速度环）
    double steering = -angular_vel_pid_update(angular_vel_body, target_angular_vel);
    
     RCLCPP_INFO(get_logger(), "steering: %.3f rad\n", steering);
    // 8. 发布控制命令
    if (std::isfinite(steering)) {
      publish_control_command(steering);
    } else {
      RCLCPP_ERROR(get_logger(), "Invalid steering output: %f", steering);
    }
  }

  Eigen::Vector3d apply_coordinate_transform(const sensor_msgs::msg::Imu::SharedPtr &msg) {
    Eigen::Vector3d angular_raw(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
    return rotation_matrix_ * angular_raw;
  }

  double get_yaw_from_imu(const sensor_msgs::msg::Imu::SharedPtr &msg) {
    static double prev_yaw;
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // 如果是第一次调用，直接返回当前 yaw
    if (is_first_call) {
        prev_yaw = yaw;
        is_first_call = false;
        return yaw;
    }

    // 计算当前 yaw 与前一个 yaw 的差值
    double delta = (yaw - prev_yaw);
    

    // // 如果跳变超过 π，修正 yaw（保证连续性）
    // if (delta > M_PI) {
    //     yaw -= 2 * M_PI;  // 正向跳变修正（例如 π → -π）
    // } else if (delta < -M_PI) {
    //     yaw += 2 * M_PI;  // 负向跳变修正（例如 -π → π）
    // }

    prev_yaw = yaw;  // 更新前一个 yaw
    return yaw;

  }

  double angle_pid_update(double current_angle) {
    double dt = (now() - angle_pid_.last_time).seconds();
    if (dt <= 0) return 0.0;
    
    double error = target_angle_ - current_angle;
    if(fabs(error)<0.02) return 0.0;

    // 积分项
    angle_pid_.integral += error * dt;
    angle_pid_.integral = std::clamp(angle_pid_.integral,
                                    -angle_pid_.integral_limit,
                                    angle_pid_.integral_limit);
    
    // 微分项
    double derivative = (error - angle_pid_.last_error);
    
    // PID计算
    double output = angle_pid_.kp * error 
                   + angle_pid_.ki * angle_pid_.integral 
                   + angle_pid_.kd * derivative;
    
    // 更新状态
    angle_pid_.last_error = error;
    angle_pid_.last_time = now();
    
    return output;
  }

  double angular_vel_pid_update(double current_vel, double target_vel) {
    double dt = (now() - angular_vel_pid_.last_time).seconds();
    if (dt <= 0) return 0.0;
    
    double error = target_vel - current_vel;
    
    // 积分项
    angular_vel_pid_.integral += error * dt;
    angular_vel_pid_.integral = std::clamp(angular_vel_pid_.integral,
                                          -angular_vel_pid_.integral_limit,
                                          angular_vel_pid_.integral_limit);
    
    // 微分项
    double derivative = (error - angular_vel_pid_.last_error) / dt;
    
    // PID计算
    double output = angular_vel_pid_.kp * error 
                   + angular_vel_pid_.ki * angular_vel_pid_.integral 
                   + angular_vel_pid_.kd * derivative;
    
    // 更新状态
    angular_vel_pid_.last_error = error;
    angular_vel_pid_.last_time = now();
    
    return std::clamp(output, -max_steering_, max_steering_);
  }

  void publish_control_command(double steering) {
    static auto last_pub_time = this->now();
    static auto cmd_msg = geometry_msgs::msg::Twist();
    
    // 更新消息内容
    target_speed_=g_current_linear_velocity;
    cmd_msg.linear.x = target_speed_-fabs(steering)/1.25*target_speed_;
    // if(cmd_msg.linear.x <0.03)
    // {
    //    cmd_msg.linear.x=0;
    // }
    //  if(cmd_msg.linear.x >0.03)
    // {
    //    cmd_msg.linear.x=0.05;
    // }
    cmd_msg.angular.z = steering/2.6;
    
    // 频率控制 (50Hz)
    if ((this->now() - last_pub_time).seconds() >= 0.25) {
      cmd_pub_->publish(cmd_msg);
      last_pub_time = this->now();
    } 
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) 
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    
    for (const auto &param : parameters) {
      try {
        if (param.get_name() == "target_speed") {
          target_speed_ = param.as_double();
          RCLCPP_INFO(get_logger(), "Updated target speed: %.2f", target_speed_);
        }
        else if (param.get_name() == "max_steering") {
          max_steering_ = param.as_double();
          RCLCPP_INFO(get_logger(), "Updated max steering: %.2f", max_steering_);
        }
        else if (param.get_name() == "target_angle") {
          target_angle_ = param.as_double();
          RCLCPP_INFO(get_logger(), "Updated target angle: %.3f rad", target_angle_);
        }
        else if (param.get_name() == "angle_kp") {
          angle_pid_.kp = param.as_double();
        }
        else if (param.get_name() == "angle_ki") {
          angle_pid_.ki = param.as_double();
        }
        else if (param.get_name() == "angle_kd") {
          angle_pid_.kd = param.as_double();
        }
        else if (param.get_name() == "angular_vel_kp") {
          angular_vel_pid_.kp = param.as_double();
        }
        else if (param.get_name() == "angular_vel_ki") {
          angular_vel_pid_.ki = param.as_double();
        }
        else if (param.get_name() == "angular_vel_kd") {
          angular_vel_pid_.kd = param.as_double();
        }
      } catch (const rclcpp::ParameterTypeException &e) {
        result.successful = false;
        result.reason = e.what();
      }
    }
    return result;
  }

  void update_filter(std::deque<double>& buffer, double new_value, size_t window_size) {
    buffer.push_back(new_value);
    if(buffer.size() > window_size) {
      buffer.pop_front();
    }
  }

  double get_filtered_value(const std::deque<double>& buffer) {
    if(buffer.empty()) return 0.0;
    return std::accumulate(buffer.begin(), buffer.end(), 0.0) / buffer.size();
  }

  // 成员变量
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_; // 新增订阅者
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_handler_;
  
  PIDParams angle_pid_;
  PIDParams angular_vel_pid_;
  
  std::deque<double> angle_filter_;
  std::deque<double> angular_vel_filter_;
  size_t angle_filter_window_;
  size_t angular_vel_filter_window_;
  
  Eigen::Matrix3d rotation_matrix_;
  std::string imu_topic_;
  std::string cmd_vel_topic_;
  double target_speed_;
  double max_steering_;
  double target_angle_;
  bool first_imu_received_;
  bool two_imu_received_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StraightController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

