#include "rclcpp/rclcpp.hpp"
#include <point_cloud_filter/PointCloudManager.hpp>
#include <memory>
#include "std_msgs/msg/string.hpp"

PointCloudManager::PointCloudManager() : Node("PointCloudManager", rclcpp::NodeOptions().use_intra_process_comms(true))
{

  nodeHandle = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  sub_topic_0 = "/livox/lidar";
  pub_topic_0 = "/output_filter";
  pub_topic_1 = "/output_filter_downsample";


  z_limit_min = 0.0f;
  z_limit_max = 1.8f;
  
  x_limit_min_1 = -0.6f;
  x_limit_max_1 = 0.3f;
  y_limit_min_1 = -0.3f;
  y_limit_max_1 = 1.0f;

  x_limit_min_2 = -0.6f;
  x_limit_max_2 = 0.3f;
  y_limit_min_2 = -1.0f;
  y_limit_max_2 = 0.3f;

  vloex_limit_x = 0.1f;
  vloex_limit_y = 0.1f;
  vloex_limit_z = 0.1f;

  radius_search = 0.2f;
  min_neighbors_in_radius = 10.0f;

  pcl_publisher_0 = nodeHandle->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic_0, rclcpp::SensorDataQoS());

  pcl_sub_0 = nodeHandle->create_subscription<sensor_msgs::msg::PointCloud2>(
      sub_topic_0, rclcpp::SensorDataQoS(), std::bind(&PointCloudManager::pclCallback, this, std::placeholders::_1));
}

PointCloudManager::~PointCloudManager()
{
}

void PointCloudManager::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  //***********************子采样,减少点云数量************************
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; //原始的点云的数据格式
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);          //指针转换
  pcl::PCLPointCloud2 cloud_filtered;                   //存储滤波后的数据格式

  pcl_conversions::toPCL(*cloud_msg, *cloud); // 转化为PCL中的点云的数据格式
                                              // 进行一个滤波处理
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;    //实例化滤波
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(vloex_limit_x, vloex_limit_y, vloex_limit_z); //设置体素网格的大小
  sor.filter(cloud_filtered);                                   //存储滤波后的点云

  pcl::PointCloud<pcl::PointXYZ> icp_cloud;
  pcl::fromPCLPointCloud2(cloud_filtered, icp_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_again_z(new pcl::PointCloud<pcl::PointXYZ>); //存储滤波后的数据格式
  //***********************Z轴范围裁剪************************
  pass.setInputCloud(icp_cloud.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_limit_min, z_limit_max);
  //pass.setFilterLimitsNegative (true);
  pass.filter(*cloud_filtered_again_z);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_radius_outlier(new pcl::PointCloud<pcl::PointXYZ>);
  if (cloud_filtered_again_z->size() > 0) //如果包含的点数量大于0,进行清除孤立点过滤处理
  {
    // build the filter
    outrem.setInputCloud(cloud_filtered_again_z);
    outrem.setRadiusSearch(radius_search);
    // outrem.setMinNeighborsInRadius(min_neighbors_in_radius);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter(*cloud_filtered_radius_outlier);

    sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
    pcl::toROSMsg(*cloud_filtered_radius_outlier, output);
    pcl_publisher_0->publish(output);
  }
  else
  {
    sensor_msgs::msg::PointCloud2 output; //声明的输出的点云的格式
    pcl::toROSMsg(*cloud_filtered_again_z, output);
    pcl_publisher_0->publish(output);
  }
}
