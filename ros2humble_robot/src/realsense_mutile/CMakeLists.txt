cmake_minimum_required(VERSION 3.5)
project(realsense_mutile)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
#find_package(diagnostic_updater REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
)

# Install launch files
install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME}
    )



# Ament exports
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
