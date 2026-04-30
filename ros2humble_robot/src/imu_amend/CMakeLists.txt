cmake_minimum_required(VERSION 3.8)
project(imu_amend)

# 默认构建选项
option(BUILD_IMU_AMEND "Build IMU amend package" ON)

if(BUILD_IMU_AMEND)
  # 设置C++标准
  set(CMAKE_CXX_STANDARD 17)
  
  # 查找依赖（新增tf2相关依赖）
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2 REQUIRED)                  # 新增
  find_package(tf2_geometry_msgs REQUIRED)    # 新增
  find_package(Eigen3 REQUIRED)

  # 包含目录
  include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}                      # 新增
  )

  # 添加可执行文件
  add_executable(imu_amend_node 
    src/imu_amend_node.cpp
  )

  # 链接库（添加tf2链接）
  ament_target_dependencies(imu_amend_node
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2                      # 新增
    tf2_geometry_msgs        # 新增
  )

  # 安装目标
  install(TARGETS imu_amend_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # 安装包含目录
  install(DIRECTORY include/
    DESTINATION include
  )

  # 安装配置和启动文件
  install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()