# rover_bringup ROS2 底层通讯驱动

 **Copyright (c) 2024 rover Robot**

当前版本协议：V3.3.0

![RUNOOB 图标](./logo2.png)

 **NOTE:**  
* 驱动使用私有通讯协议
* 带线速度和角速度校准调节比例参数
* 带有基于里程计odom坐标发布功能
* 需要安装ROS环境
* 通讯速率高达500HZ
* (使用IMU 融合)需要安装基础包
* 软速度平滑
* 
## 依赖包安装 -

设置端口绑定

`$ cd /rover_driver/startup/` 
    
`$ sudo chmod a+x initenv.sh` 

`$ ./initenv.sh`

安装依赖包

`sudo apt-get update`

`sudo apt install ros-humble-joint-state-publisher`

`sudo apt install ros-humble-xacro`

`sudo apt-get install ros-humble-imu-tools`

`sudo apt-get install ros-humble-robot-localization`

## 启动文件

启动驱动包
``` linux
ros2 launch rover_driver rover_driver.launch.py
```

# rover Bringup API
## 发布的话题
/imu/data_raw ([sensor_msgs/msg/Imu](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html))  
- 主题包括来自 IMU 的惯性数据。

/raw_odom ([nav_msgs/msg/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html))  
- 基于编码器的的里程表信息

/rover_driver/battery_state (rover_msgs/msg/roverBmsStatus)
- 电池电压和电量状态

/rover_driver/rc_state (rover_msgs/msg/roverRCStatus)
- 反馈遥控数据（包括遥杆、开关、旋钮数据）

/rover_driver/rc_state (rover_msgs/msg/ChassisStatus)
- 反馈底盘状态（电机、急停、防碰撞等）

## 订阅的话题
/cmd_vel ([geometry_msgs/msg/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html))
- 控制机器人平移线速度(单位:m/s)和旋转角速度(单位:rad/s)


## 参数设置 - Parameters
设置文件所在位置`/rover_driver/launch/rover_driver.launch.py`

- ~serial_port_baud (int, default: 230400)  
    设置串口波特率
- ~serial_port_baud (string, default: /dev/rover)  
    设置串口号

- ~pub_odom_tf (bool, default: false)  
    是否发布基于里程计odom的TF转换
- ~linear_scale (double, default: 1.0)  
    里程计线速度校准比例
- ~angular_scale (double, default: 1.0)  
    里程计角速度校准比例
  
- ~imu_publish (string, default: false)  
    是否发布板载imu信息
- ~cmd_vel_sub_timeout (double, default: 1.0)  
    接收cmd_vel超时时间，超时后速度置0
- ~imu_calibrate_gyro (string, default: true)  
    是否校准陀螺仪
- ~imu_cailb_samples (int, default: 300)  
    校准陀螺仪偏差的测量次数（越大精度会提高，相对的启动校准时间也增加）
    
IMU注意事项：首次订阅校准陀螺仪会有延迟并且不要动机器人
    
