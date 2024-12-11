//本节点输入为前后左右四路激光点云,输出前后左右障碍物最近距离和前后纠偏误差

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <ros/spinner.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
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

#include "cloud_preprocess.h"
#include <vector>
#include <algorithm>

using namespace std;
//根据商品车转运机器人内部的点云进行判断
//前方剩余间隔、后方剩余间隔
ros::NodeHandle *nh;
float agv_width=3, agv_length=5.99, agv_inside_width=2.105; 
int agv_in_flag=0, agv_all_in_flag=0,agv_all_in_flag_pre=0;
float inside_front_dis=999, inside_back_dis=999;
string move_dir="";

// 根据商品车转运机器人内部的点云进行判断
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    //static TDataFilter filter_err(10), f_inside_front_dis(2), f_inside_back_dis(2);
    static PointCloud a_cloud;
    static int a_count=0;

    PointCloud cloud;
    //将 ROS 的消息类转换为 PCL 的点云数据结构
    pcl::fromROSMsg(*c_msg, cloud);

    a_cloud+=cloud;
    a_cloud.header=cloud.header;
    a_count++;
    // printf("1111111111111111111111\n");
    if(a_count<2)  return;    

    // float inside_err=999, inside_left_dis=999, inside_right_dis=999;
    inside_front_dis=inside_back_dis=999;
    agv_in_flag=agv_all_in_flag=false;

    if(cloud.size()>20)
    {
        //  分为前半和后半部分
        //根据x,y,z轴的极值大小进行点云滤波
        PointCloud front_cloud=Cloud_PassThrough(a_cloud, 0, agv_length*0.5, 0, 0, 0, 0);
        PointCloud back_cloud=Cloud_PassThrough(a_cloud, -agv_length*0.5, 0, 0, 0, 0, 0);

        //  分别计算前后两部分点云的宽度和长度
        PointType cloud_min, cloud_max;
        float front_width=0,  front_length=0;
        float back_width=0,  back_length=0;
        //计算前部分点云宽度
        if(front_cloud.size()>0)
        {  
            pcl::getMinMax3D(front_cloud, cloud_min, cloud_max);
            front_width=cloud_max.y-cloud_min.y,  front_length=cloud_max.x-cloud_min.x;
        }
        //计算后部分的点云宽度
        if(back_cloud.size()>0)
        {        
            pcl::getMinMax3D(back_cloud, cloud_min, cloud_max);
            back_width=cloud_max.y-cloud_min.y,  back_length=cloud_max.x-cloud_min.x;  
        }

        bool front_agv_in=front_width>1.0;
        bool back_agv_in=back_width>1.0;  
        agv_in_flag=front_agv_in || back_agv_in; 
        if(agv_in_flag) 
        {
            pcl::getMinMax3D(a_cloud, cloud_min, cloud_max);
            inside_front_dis=agv_length*0.5-cloud_max.x;       //前方剩余间隔
            inside_back_dis=agv_length*0.5+cloud_min.x;       //后方剩余间隔  
            //flag
            bool front_pick_flag= (inside_back_dis<1.8 || inside_front_dis>0.05) && back_agv_in && move_dir=="front";
            bool back_pick_flag = (inside_front_dis<1.8 || inside_back_dis>0.05) && front_agv_in && move_dir=="back";
            agv_all_in_flag=front_pick_flag || back_pick_flag;
        }
    }

    a_cloud.clear();
    a_count = 0;   
}

//运动方向回调函数
void MoveDirCallback(const std_msgs::String::ConstPtr &msg)
{
    move_dir=msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_calculation_inside");
    nh=new ros::NodeHandle("~");
    
    TNodeCheck *nodecheck=new TNodeCheck(nh, "node_rate");

    ros::Subscriber cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_inside_cloud", 10, &PointCloudCallback);
    ros::Subscriber move_dir_sub = nh->subscribe<std_msgs::String>("/move_dir", 10, &MoveDirCallback);
    
    ros::Publisher pub_agv_in = nh->advertise<std_msgs::Int32MultiArray>("agv_in_flag", 10);
    ros::Publisher pub_car_dis = nh->advertise<std_msgs::Float32MultiArray>("car_dis", 10);

    ros::Rate looprate(20);
    while (ros::ok())
    {
        nodecheck->Find("node_rate")->Beat();
 
        std_msgs::Int32MultiArray agv_in_msg;
        agv_in_msg.data.clear();
        //flag
        //agv_in_flag=front_agv_in || back_agv_in; 
        agv_in_msg.data.push_back(agv_in_flag);
        // agv_all_in_flag=front_pick_flag || back_pick_flag;
        agv_in_msg.data.push_back(agv_all_in_flag);
        pub_agv_in.publish(agv_in_msg);

        std_msgs::Float32MultiArray car_dis_msg;
        car_dis_msg.data.clear();
        //前方剩余间隔
        car_dis_msg.data.push_back(inside_front_dis);
        //后方剩余间隔
        car_dis_msg.data.push_back(inside_back_dis);
        // printf("inside=%.2f\n",inside_front_dis);
        pub_car_dis.publish(car_dis_msg);

        ros::spinOnce();
        looprate.sleep();
    } 
    
    ros::shutdown();
    return 0;
}