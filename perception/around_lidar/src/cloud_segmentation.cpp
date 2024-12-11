//本节点输入为前后左右四路激光点云,输出前后左右内五路激光雷达和前后强度滤波之后的点云


#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <ros/spinner.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include "pointcloud_type.h"
#include "common/public.h"
#include <mqtt_comm/task.h>

#include "cloud_preprocess.h"
#include <vector>
#include <algorithm>

using namespace std;

//四个激光雷达点云合并、agv 框图、范围滤波后分割为前后左右四组点云,发布出去

ros::NodeHandle *nh;

ros::Publisher pub_cloud_front, pub_cloud_inside, pub_cloud_back, pub_cloud_left, pub_cloud_right,pub_cloud_all;
ros::Publisher pub_cloud_front_marker, pub_cloud_back_marker;  // 发布前后marker点云

ros::Publisher pub_polygon, pub_car_shape; // agv 框图
PointCloud cloud[4];
bool cloud_flag[4];

float max_xy_range=50, max_z=0.1, min_z=-0.6;  
float agv_width=3, agv_length=5.99, agv_inside_width=2.105, car_width=0.5;  
float lidar_dead_leftright=0.05, lidar_dead_frontback=0.05;
TNodeCheck *nodecheck;
// 点云回调函数：消息转换
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, int id)
{
    cloud_flag[id]=true;
    if(cloud[id].size()>0)  return;
    pcl::fromROSMsg(*msg, cloud[id]);
}

//选取范围内的数据
bool CheckInRange(float x, float y, float x_min, float x_max, float y_min, float y_max)
{
    return (x>=x_min && x<=x_max && y>=y_min && y<=y_max);
}

//  点云组合,范围滤波后分割为前后左右四组点云,发布出去
void cloud_accumulation()     
{
    // printf("%d %d %d %d\n",cloud[0].size(),cloud[1].size(),cloud[2].size(),cloud[3].size());
     //4个雷达
    for(int i=0;i<4;i++)  
    {
        char str[200];
        sprintf(str,"/cloud_tf%d/check/rslidar_rate",i+1);
        float rate=0;
        nh->getParam(str,rate);  //rate
        bool lidar_enable=rate>10;  
        //  雷达有效且无数据就应该退出
        if(!cloud_flag[i] && lidar_enable)  return;  //  雷达有效且无数据就应该退出
    }
    //将四个点云合并到一个新的点云all_cloud中，
    //并在合并后清空原始的四个点云以及更新相应的标记 cloud_flag
    PointCloud all_cloud;
    for(int i=0;i<4;i++)  
    {
        all_cloud+=cloud[i];
        cloud[i].clear();
        cloud_flag[i]=false;
    } 
    if(all_cloud.size()<10)  return;

    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(all_cloud, msgx); //将PCL的PointCloud对象all_cloud转换为ROS的PointCloud2消息msgx
    msgx.header.frame_id = "base_link";  //点云数据所在的参考坐标系
    msgx.header.stamp=ros::Time::now();  //记录消息的时间戳，
    pub_cloud_all.publish(msgx);   //发布四个点云合并后的点云

    // agv 框图
    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = msgx.header.frame_id;  //参考坐标系 
    polygon.header.stamp = ros::Time::now();  //时间戳
    //AGV的框
    polygon = GetPolygon(msgx,-agv_length*0.5,agv_length*0.5,-agv_width*0.5,agv_width*0.5);
    pub_polygon.publish(polygon);  //发布

    PointCloud inside_cloud, front_cloud,back_cloud,left_cloud,right_cloud,frontmarker_cloud,backmarker_cloud;
     //统一头信息
    inside_cloud.header=all_cloud.header;
    front_cloud.header=all_cloud.header;
    back_cloud.header=all_cloud.header;
    left_cloud.header=all_cloud.header;
    right_cloud.header=all_cloud.header;

    frontmarker_cloud.header=all_cloud.header;
    backmarker_cloud.header=all_cloud.header;

    nh->getParam("lidar_dead_leftright", lidar_dead_leftright);
    nh->getParam("lidar_dead_frontback", lidar_dead_frontback);

    for(auto it:all_cloud)
    {   ////选取范围内的数据CheckInRange
        //前后标记物检测
        bool frontmarker_flag=CheckInRange(it.x, it.y, agv_length*0.5+0.05, 15, -agv_width*0.5-1.5, agv_width*0.5+1.5);
        bool backmarker_flag=CheckInRange(it.x, it.y, -15, -agv_length*0.5-0.05, -agv_width*0.5-1.5, agv_width*0.5+1.5);
        if (frontmarker_flag)  frontmarker_cloud.push_back(it);
        else if(backmarker_flag)  backmarker_cloud.push_back(it);
        //内部区域检测
        bool inside_flag=CheckInRange(it.x, it.y, -agv_length*0.5,agv_length*0.5,-agv_inside_width*0.5+0.15,agv_inside_width*0.5-0.15);
        if (inside_flag)  
        {
            inside_cloud.push_back(it);
            continue;
        } 
        //前方区域检测
        bool front_flag=CheckInRange(it.x, it.y,  agv_length*0.5+lidar_dead_frontback, max_xy_range, -agv_width*0.5, agv_width*0.5);
        if (front_flag)  
        {
            front_cloud.push_back(it);
            continue;
        }
        //后方区域检测
        bool back_flag=CheckInRange(it.x, it.y,  -max_xy_range, -agv_length*0.5-lidar_dead_frontback, -agv_width*0.5, agv_width*0.5);
        if (back_flag)  
        {
            back_cloud.push_back(it);
            continue;
        }  
        //左侧区域检测
        bool left_flag=CheckInRange(it.x, it.y,  -agv_length*0.5, agv_length*0.5, agv_width*0.5+lidar_dead_leftright, max_xy_range);
        if (left_flag)  
        {
            left_cloud.push_back(it);
            continue;
        } 
        //右侧区域检测
        bool right_flag=CheckInRange(it.x, it.y,  -agv_length*0.5, agv_length*0.5, -max_xy_range, -agv_width*0.5-lidar_dead_leftright);
        if (right_flag)  
        {
            right_cloud.push_back(it);
        } 
    }
     
    //数据类型转换、定义参考坐标系、时间戳、发布
    pcl::toROSMsg(inside_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp=ros::Time::now();
    pub_cloud_inside.publish(msgx);

    pcl::toROSMsg(front_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp=ros::Time::now();
    pub_cloud_front.publish(msgx);

    pcl::toROSMsg(back_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp = ros::Time::now();
    pub_cloud_back.publish(msgx);

    pcl::toROSMsg(left_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp =ros::Time::now();
    pub_cloud_left.publish(msgx);

    pcl::toROSMsg(right_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp=ros::Time::now();
    pub_cloud_right.publish(msgx);

    //  分割前后带标记物点云
    // PointCloud frontmarker_cloud =  Cloud_PassThrough(all_cloud, agv_length*0.5+0.05, 15, -agv_width*0.5-0.5, agv_width*0.5+0.5, min_z, max_z);
    //根据点云强度进行过滤
    frontmarker_cloud=Cloud_IntensityFilter(frontmarker_cloud,130);
    // PointCloud backmarker_cloud = Cloud_PassThrough(all_cloud, -10, -agv_length*0.5-0.05, -agv_width*0.5-1, agv_width*0.5+1, min_z, max_z);
    backmarker_cloud=Cloud_IntensityFilter(backmarker_cloud,130);

    pcl::toROSMsg(frontmarker_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp = ros::Time::now();
    pub_cloud_front_marker.publish(msgx);

    pcl::toROSMsg(backmarker_cloud, msgx);
    msgx.header.frame_id = "base_link";
    msgx.header.stamp = ros::Time::now();
    pub_cloud_back_marker.publish(msgx);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_segmentation");
    nh=new ros::NodeHandle("~");
    nodecheck = new TNodeCheck(nh, "node_rate");

    pub_cloud_all = nh->advertise<sensor_msgs::PointCloud2>("agv_all_cloud", 10);
    pub_cloud_inside = nh->advertise<sensor_msgs::PointCloud2>("agv_inside_cloud", 10);
    pub_cloud_front = nh->advertise<sensor_msgs::PointCloud2>("agv_front_cloud", 10);
    pub_cloud_back = nh->advertise<sensor_msgs::PointCloud2>("agv_back_cloud", 10);
    pub_cloud_left = nh->advertise<sensor_msgs::PointCloud2>("agv_left_cloud", 10);
    pub_cloud_right = nh->advertise<sensor_msgs::PointCloud2>("agv_right_cloud", 10);

    pub_cloud_front_marker = nh->advertise<sensor_msgs::PointCloud2>("agv_frontmarker_cloud", 10);
    pub_cloud_back_marker = nh->advertise<sensor_msgs::PointCloud2>("agv_backmarker_cloud", 10);

    // agv 框图
    pub_polygon = nh->advertise<geometry_msgs::PolygonStamped>("car_region", 10);

    ros::Subscriber cloud_sub[4];
    for(int i=0;i<4;i++)
    {
        char topicname[100];
        sprintf(topicname,"/cloud_tf%d/cloud_tf", i+1);
        cloud_sub[i]=nh->subscribe<sensor_msgs::PointCloud2>(topicname, 10, boost::bind(&PointCloudCallback,_1, i));    
    } 

   
    ros::Rate looprate(20);
    while (ros::ok())
    {    
        nodecheck->Find("node_rate")->Beat();
        cloud_accumulation();      

        ros::spinOnce();
        looprate.sleep();
    } 
    
    ros::shutdown();
    return 0;
}