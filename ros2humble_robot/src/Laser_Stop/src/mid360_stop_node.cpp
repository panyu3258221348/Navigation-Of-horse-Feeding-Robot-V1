#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
using std::placeholders::_1;
using namespace std::chrono_literals;

class Mid360StopNode : public rclcpp::Node {
  bool mid360_flag = false;

public:
  Mid360StopNode() : Node("lader_subscriber") {
    mid360_subscription_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar/pointcloud", 10,
            std::bind(&Mid360StopNode::topic_callback, this, _1));
    stopnode_publisher_ =
        this->create_publisher<std_msgs::msg::String>("stop_node", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Mid360StopNode::timer_callback, this));
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    mid360_flag = false;
     RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message");
    size_t num_points = msg->width * msg->height;
    // RCLCPP_INFO(this->get_logger(), "Number of points: %zu", num_points);
    size_t point_strp = msg->point_step;
    size_t row_step = msg->row_step;
    const uint8_t *data = msg->data.data();
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    const float front_distance_threshold = 0.55;
    const float front_height_threshold = 0.4;
    const float rear_distance_threshold = 0.1;
    const float zero = 0.0;
    size_t index = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++index) {
      if (*iter_x <= front_distance_threshold &&
          fabs(*iter_z) <= front_height_threshold &&
          fabs(*iter_y) <= rear_distance_threshold && *iter_x >= zero &&
          *iter_y != zero && *iter_z != zero) {
        mid360_flag = true;
        if (index % 100 == 0) {
          RCLCPP_INFO(this->get_logger(), "Point %zu:(%f,%f,%f)", index,
                      *iter_x, *iter_y, *iter_z);
        }
        break;
      }
      index++;
    }

    // for (size_t i = 0; i < msg.ranges.size(); ++i) {
    //   float angle = (msg.angle_min + i * msg.angle_increment) * 180 / M_PI;
    //   if (abs(angle) > 90 && msg.ranges[i] < 0.3) {
    //     laser_flag = true;
    //     break;
    //   }
    // }
  }

  void timer_callback() {
    auto message = std_msgs::msg::String();
    if (mid360_flag) {
      message.data = "laser_stop";
      stopnode_publisher_->publish(message);
    } else {
      message.data = "no_people";
      stopnode_publisher_->publish(message);
    }
    // RCLCPP_INFO(this->get_logger(), "message: %s", message.data.c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      mid360_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stopnode_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360StopNode>());
  rclcpp::shutdown();
  return 0;
}
