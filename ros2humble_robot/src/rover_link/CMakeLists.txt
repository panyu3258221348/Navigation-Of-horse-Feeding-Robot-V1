cmake_minimum_required(VERSION 3.5)
project(rover_link)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions("-Wall -Wextra -Werror")

# find dependencies
find_package(mavros REQUIRED)

find_package(mavros_msgs REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rover_interfaces REQUIRED)
find_package(serial REQUIRED)

find_package(nav2_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
################################################################################
# Build
################################################################################
include_directories(
  include
  include/${PROJECT_NAME}
)

set(DEPENDENCIES
  "mavros"
  "mavros_msgs"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "rover_interfaces"
  "serial"
  "diagnostic_msgs"
  "nav2_msgs"
  "rclcpp_action"
  "rclcpp_components"
  
)


add_executable(offboard_nav_manager src/offboard_nav_manager.cpp)
ament_target_dependencies(offboard_nav_manager mavros_msgs ${DEPENDENCIES})

add_executable(offboard_serial src/offboard_serial.cpp src/CRC16.cpp)
ament_target_dependencies(offboard_serial mavros_msgs ${DEPENDENCIES})

add_executable(rover_model_switcher src/rover_model_switcher.cpp src/CRC16.cpp)
ament_target_dependencies(rover_model_switcher mavros_msgs ${DEPENDENCIES})

add_executable(offboard_imu src/offboard_imu.cpp src/CRC16.cpp)
ament_target_dependencies(offboard_imu mavros_msgs ${DEPENDENCIES})



add_executable(offboard_map_exchange src/offboard_map_exchange.cpp)
ament_target_dependencies(offboard_map_exchange mavros_msgs ${DEPENDENCIES})


add_executable(offboard_filter_msg src/offboard_filter_msg.cpp)
ament_target_dependencies(offboard_filter_msg mavros_msgs ${DEPENDENCIES})




install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS  
  ${EXECUTABLE_NAME} 
  
  offboard_nav_manager
  offboard_serial
  offboard_imu
  offboard_map_exchange
  offboard_filter_msg
  rover_model_switcher
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_package()