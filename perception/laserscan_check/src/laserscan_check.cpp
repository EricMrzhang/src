#define PCL_NO_PRECOMPILE

#include <common/public.h>

#include <Eigen/Core>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <laser_geometry/laser_geometry.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include "cloud_preprocess.h"

//  本程序输入为线激光雷达点云,经过三维点云数据转换,范围滤波和聚类处理后,获得轮胎扫描线的长度,中心位置和距离
// BUG 速度1Hz，去掉欧式聚类

using namespace std;

//ROS中的sensor_msgs::LaserScan消息转换为PCL
//中的点云格式pcl::PointCloud<pcl::PointXYZI>
void LaserScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &c_msg, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
    //  激光扫描数据投影到三维空间中
    laser_geometry::LaserProjection projection;
    sensor_msgs::PointCloud2 cloud_msg;   //表示点云数据的一种消息类型
    projection.projectLaser(*c_msg, cloud_msg);
    //将ROS的sensor_msgs::PointCloud2类型数据转换为PCL的pcl::PointCloud<T>类型数据
    pcl::fromROSMsg(cloud_msg, cloud);  //
}

class Laserscan_Check
{
private:
    ros::NodeHandle *nh;
    ros::Subscriber laserscan_sub[4];
    ros::Publisher wheel_dis_pub, cloud_pub[4], distancemarker_pub;

    TNodeCheck *nodecheck;

public:
    float box_x_min = 0.05, box_x_max = 0.4;
    float box_y_min = -0.5, box_y_max = 0.5;

    geometry_msgs::Point center_point[4];
    //于发布轮距数据和距离标记的可视化信息
    Laserscan_Check()
    {
        //创建一个 ros::NodeHandle 的实例，并将其地址赋值给指针 nh
        nh = new ros::NodeHandle("~");

        string scan_lidar_type;
        //根据不同的激光雷达类型订阅不同的激光扫描
        //（sensor_msgs::LaserScan）主题
        nh->getParam("/scanlidar",scan_lidar_type);
        if(scan_lidar_type=="c200")
        {
            laserscan_sub[0] = nh->subscribe<sensor_msgs::LaserScan>("/c200_lidar_node1/scan", 10, &Laserscan_Check::LaserScanCallback1, this);
            laserscan_sub[1] = nh->subscribe<sensor_msgs::LaserScan>("/c200_lidar_node2/scan", 10, &Laserscan_Check::LaserScanCallback2, this);
            laserscan_sub[2] = nh->subscribe<sensor_msgs::LaserScan>("/c200_lidar_node3/scan", 10, &Laserscan_Check::LaserScanCallback3, this);
            laserscan_sub[3] = nh->subscribe<sensor_msgs::LaserScan>("/c200_lidar_node4/scan", 10, &Laserscan_Check::LaserScanCallback4, this);
        }
        else
        {
            laserscan_sub[0] = nh->subscribe<sensor_msgs::LaserScan>("/olelidar1/scan", 10, &Laserscan_Check::LaserScanCallback1, this);
            laserscan_sub[1] = nh->subscribe<sensor_msgs::LaserScan>("/olelidar2/scan", 10, &Laserscan_Check::LaserScanCallback2, this);
            laserscan_sub[2] = nh->subscribe<sensor_msgs::LaserScan>("/olelidar3/scan", 10, &Laserscan_Check::LaserScanCallback3, this);
            laserscan_sub[3] = nh->subscribe<sensor_msgs::LaserScan>("/olelidar4/scan", 10, &Laserscan_Check::LaserScanCallback4, this);
        }
         //发布点云数据到四个不同的主题（cloud1_filter、cloud2_filter、cloud3_filter、cloud4_filter）
        cloud_pub[0] = nh->advertise<sensor_msgs::PointCloud2>("cloud1_filter", 10);
        cloud_pub[1] = nh->advertise<sensor_msgs::PointCloud2>("cloud2_filter", 10);
        cloud_pub[2] = nh->advertise<sensor_msgs::PointCloud2>("cloud3_filter", 10);
        cloud_pub[3] = nh->advertise<sensor_msgs::PointCloud2>("cloud4_filter", 10);
         //发布std_msgs::Float32MultiArray类型的消息到名为"wheel_distance"的主题上
        //于发布轮距数据和距离标记的可视化信息
        wheel_dis_pub = nh->advertise<std_msgs::Float32MultiArray>("wheel_distance", 10);
        distancemarker_pub = nh->advertise<visualization_msgs::MarkerArray>("distance_markers", 10);

        nodecheck = new TNodeCheck(nh, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(10);
    }

    void LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg);

    void CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void WheelDistancePublish();
};

 //点云处理：点云滤波、点云中心点坐标
void Laserscan_Check::CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg) // pcl::PointCloud<pcl::PointXYZI> cloud, geometry_msgs::Point &point)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;  ////点云对象
    LaserScan2PointCloud(c_msg, cloud);   //消息转换
    //根据给定的x、y、z坐标范围来过滤点云
    cloud = Cloud_PassThrough(cloud, box_x_min, box_x_max, box_y_min, box_y_max, 0, 0);
    //移除点云中那些在给定半径内邻居点数量小于指定阈值的点
    cloud = Cloud_RadiusFilter(cloud, 0.05, 10);

    // printf("%d=%d\n", id, cloud.size());
    //表示点云数据的消息类型
    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(cloud, msgx);  //将PCL点云数据转换为ROS消息
    msgx.header.frame_id = c_msg->header.frame_id;
    cloud_pub[id].publish(msgx);   //发布点云id
    
    pcl::PointXYZI minpoint, maxpoint;
    //所有点中最小的x值，y值，z值，输出当前点云所有点中最大的x值，y值，z值。
    pcl::getMinMax3D(cloud, minpoint, maxpoint);
    center_point[id].x = -1;   
    center_point[id].y = -1;   //  not reach
    center_point[id].z = 0;  
    //中心点坐标
    if(cloud.points.size()>5)
    {
        center_point[id].x = (minpoint.x+maxpoint.x)*0.5;
        center_point[id].y = (minpoint.y+maxpoint.y)*0.5;
        center_point[id].z = maxpoint.y-minpoint.y;
        if(id==3 || id==0)  center_point[id].y*=-1; 
    }
}

//发布轮距以及//输出四个雷达工作状态、中心点x坐标和y坐标的字符

void Laserscan_Check::WheelDistancePublish()
{
    bool working_flag[4];   //  判断雷达是否工作
    //判断四个雷达是否正在工作
    for(int i=0;i<4;i++)
    {
        float value=0; 
        char paramname[200];
        sprintf(paramname,"/olelidar%d/node_rate",i+1);
        nh->getParam(paramname, value);
        working_flag[i]=(value>5);
    }

    //检查两对雷达（雷达1与雷达3，雷达0与雷达2）是否同时处于工作或未工作状态之后，
    //根据雷达工作的状态不同得到对应的中心点坐标
    //计算轮距
    if(working_flag[1]!=working_flag[3])  
    {
        if(!working_flag[1])  center_point[1]=center_point[3];
        else if(!working_flag[3])  center_point[3]=center_point[1];
    }
    if(working_flag[0]!=working_flag[2])  
    {
        if(!working_flag[0])  center_point[0]=center_point[2];
        else if(!working_flag[2])  center_point[2]=center_point[0];
    }
    //保存并发布轮距
    std_msgs::Float32MultiArray distance;  //用于发布多维数组类型
    for(int i=0;i<4;i++)
    {
        distance.data.push_back(center_point[i].x);
        distance.data.push_back(center_point[i].y);
        distance.data.push_back(center_point[i].z);
    }
    wheel_dis_pub.publish(distance);    //发布轮距

    visualization_msgs::MarkerArray markerarray; //在RViz中发布多个Marker的数组
    visualization_msgs::Marker marker; 
    //marker属性设置  
    marker.header.stamp=ros::Time::now();
    marker.ns = "distance_markers";
    marker.lifetime = ros::Duration(0.4);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.pose.position.z = 2;
    marker.scale.z = 0.5;
    marker.pose.position.x=-0.5;

    for(int i=0;i<4;i++)
    {
        char text[100];  //定义一个字符数组text用于存储临时文本。
        sprintf(text, "scan%d", i+1);
        marker.header.frame_id = text;
        marker.id = i;  //每个雷达的标记都有一个唯一的ID。
        //输出雷达工作状态、中心点x坐标和y坐标的字符串
        sprintf(text, "wrk: %d\nx:%.3f\ny:%.3f", working_flag[i], center_point[i].x, center_point[i].y);
        marker.text = text;  //字符串被赋值给marker.text
        markerarray.markers.push_back(marker);
    }
    //发布marker
    distancemarker_pub.publish(markerarray);

    nodecheck->Find("node_rate")->Beat();
}

//雷达扫描1回调函数
void Laserscan_Check::LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(0, c_msg);
    // printf("aaaaaaaa\n");
}

//雷达扫描2回调函数
void Laserscan_Check::LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(1, c_msg);
}

//雷达扫描3回调函数
void Laserscan_Check::LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(2, c_msg);
}
//雷达扫描4回调函数
void Laserscan_Check::LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(3, c_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserscan_check"); //节点

    Laserscan_Check lk;
    ros::Rate looprate(25);

    while (ros::ok())
    {
        lk.WheelDistancePublish();   

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
