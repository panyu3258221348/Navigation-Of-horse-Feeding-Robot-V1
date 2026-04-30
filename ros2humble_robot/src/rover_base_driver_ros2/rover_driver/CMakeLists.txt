cmake_minimum_required(VERSION 3.5)
project(rover_driver)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  set(CMAKE_CXX_FLAGS_RELEASE "-O1 -Wall -g -pthread")
else()
  set(CMAKE_BUILD_TYPE Release)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ros2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# msgs interfaces
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rover_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/rover_base_driver
)

add_library(rover_base_driver
  src/rover_base_driver.cpp
  src/Serial_Async.cpp
  src/data_stream.cpp
)

ament_target_dependencies(
  rover_base_driver
    rclcpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    rover_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
)

## Build 
add_executable(rover_base_node src/rover_base_node.cpp)
target_link_libraries(rover_base_node rover_base_driver boost_thread)

install(TARGETS 
  rover_base_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch startup
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
