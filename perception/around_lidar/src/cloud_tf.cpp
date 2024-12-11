//  本节点实现四周多线激光雷达点云转换和分割,输入四路原始点云,经过滤波和坐标变换后,输出前后左右激光点云信号
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <ros/spinner.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/PolygonStamped.h>

#include "pointcloud_type.h"
#include "common/public.h"

#include "cloud_preprocess.h"
#include <vector>
#include <algorithm>

using namespace std;

// 点云滤波、坐标转换
ros::Publisher pub_cloud,pub_cloud_filter;  

float max_xy_range=50; 
float max_z=0.6, min_z=-0.8; 
float agv_width=3;

TNodeCheck *nodecheck;
string topicname;

ros::NodeHandle *nh;

//点云回调函数

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    PointCloud cloud;    
    pcl::fromROSMsg(*msg, cloud);
    // 点云滤波
    cloud = remove_infinite_points(cloud);
    if(topicname=="/rslidar_points1" || topicname=="/rslidar_points2")  
        cloud = Cloud_PassThrough(cloud, -max_xy_range, max_xy_range, -agv_width*0.5, max_xy_range, min_z, max_z);
    else 
        cloud = Cloud_PassThrough(cloud, -max_xy_range, max_xy_range, -max_xy_range, agv_width*0.5, min_z, max_z);
    
    // cloud = Cloud_PassThrough(cloud, -max_xy_range, max_xy_range, -max_xy_range, max_xy_range, min_z, max_z);
    // cloud = Cloud_IntensityFilter(cloud[id], 5);
    // 坐标转换
    cloud= CloudTf(cloud,"base_link");
    // cloud=Cloud_FastFilter(cloud, 10, 0.2);
    sensor_msgs::PointCloud2 msgx;
    // 消息转换
    pcl::toROSMsg(cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp=ros::Time::now();
    pub_cloud.publish(msgx);

    nodecheck->Find("rslidar_rate")->Beat();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_tf");
    nh=new ros::NodeHandle("~");

    nodecheck = new TNodeCheck(nh, "node_rate rslidar_rate");

    nh->getParam("input_topic",topicname);
    ros::Subscriber cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>(topicname, 10, PointCloudCallback);
    pub_cloud = nh->advertise<sensor_msgs::PointCloud2>("cloud_tf", 10);

    pub_cloud_filter = nh->advertise<sensor_msgs::PointCloud2>("cloud_filter_tf", 10);

    ros::Rate looprate(20);
    while (ros::ok())
    {
        nodecheck->Find("node_rate")->Beat();
                       
        ros::spinOnce();
        looprate.sleep();
    }

    ros::shutdown();
    return 0;
};
