
#include "rclcpp/rclcpp.hpp"
#include "rover_base_driver.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roverRos::roverBaseDriver>());
  rclcpp::shutdown();
  return 0;
}
