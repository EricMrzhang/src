#ifndef POINTCLOUD_TYPE_H
#define POINTCLOUD_TYPE_H

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>

//rslidar的点云格式

struct RsPointXYZIRT
{
    PCL_ADD_POINT4D;  //添加 XYZ+padding
    float intensity;   //点的强度信息
    std::uint16_t ring = 0;  //激光雷达的哪个环
    double timestamp = 0;   //点被捕获的时间戳
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   //// Eigen的对齐需求
} EIGEN_ALIGN16;    // 确保结构体按16字节对齐 
//通常与PCL的点类型系统一起使用，以确保点云数据的正确处理和转换
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

// typedef RsPointXYZIRT PointType;
//// 定义类型别名  
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

#endif

