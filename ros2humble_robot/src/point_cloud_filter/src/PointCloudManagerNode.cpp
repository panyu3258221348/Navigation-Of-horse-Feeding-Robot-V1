#include <rclcpp/rclcpp.hpp>
#include "point_cloud_filter/PointCloudManager.hpp"
#include <rcutils/cmdline_parser.h>


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;


  auto controller = std::make_shared<PointCloudManager>();

  executor.add_node(controller);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}

