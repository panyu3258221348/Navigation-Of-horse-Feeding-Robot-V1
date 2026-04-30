cmake_minimum_required(VERSION 3.5)
project(rover_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ros2
find_package(rosidl_default_generators REQUIRED)

# msgs interfaces
find_package(std_msgs REQUIRED)

set(
  action_files

)

set(
  msg_files
    "msg/ChassisStatus.msg"
    "msg/BatteryStatus.msg"
    "msg/RCStatus.msg"
)

set (
  srv_files
    "srv/SoundLightControl.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES 
    builtin_interfaces 
    std_msgs
  ADD_LINTER_TESTS
)

ament_package()
