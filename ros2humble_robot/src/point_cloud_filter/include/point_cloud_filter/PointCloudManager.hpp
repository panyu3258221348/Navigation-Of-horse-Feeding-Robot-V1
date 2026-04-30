#ifndef POINT_CLOUD_FILTER__POINTCLOUDMANAGER_HPP_
#define POINT_CLOUD_FILTER__POINTCLOUDMANAGER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>

using namespace std;
using std::placeholders::_1;

class PointCloudManager : public rclcpp::Node
{

public:
    explicit PointCloudManager();
    virtual ~PointCloudManager();

private:
    std::shared_ptr<rclcpp::Node> nodeHandle;

    double z_limit_min;
    double z_limit_max;

    double x_limit_min_1;
    double x_limit_max_1;

    double y_limit_min_1;
    double y_limit_max_1;

    double x_limit_min_2;
    double x_limit_max_2;

    double y_limit_min_2;
    double y_limit_max_2;

    double vloex_limit_x;
    double vloex_limit_y;
    double vloex_limit_z;

    double radius_search;
    double min_neighbors_in_radius;

    double camera_count;
    std::string sub_topic_0;
    std::string pub_topic_0;

    std::string sub_topic_1;
    std::string pub_topic_1;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor; //实例化滤波
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_0, pcl_sub_1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_0, pcl_publisher_1;

    void pclCallback_0(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloud2_Pose);
    void pclCallback_1(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloud2_Pose);
    void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pointCloud2_Pose);

};

#endif