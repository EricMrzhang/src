//本节点输入为前后左右四路激光点云,输出前后左右障碍物最近距离和前后纠偏误差

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <ros/spinner.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
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
#include "std_msgs/Float32MultiArray.h"

#include "pointcloud_type.h"
#include "common/public.h"
#include <mqtt_comm/task.h>

#include "cloud_preprocess.h"
#include <vector>
#include <algorithm>

using namespace std;

//
//本节点输入为前后两路路激光点云,输出前后障碍物最近距离
ros::NodeHandle *nh;
float obs_dis=999;
float agv_width=3, agv_length=5.99, agv_inside_width=2.105; 
mqtt_comm::task cur_task;
float remain_path_length = 0;

string dir="left";

//  接收任务指令 回调函数
void TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    // ROS_ERROR("%s\n", msg->cmd.c_str());
    if (msg->cmd.find("task") != string::npos)
    {
        cur_task = *msg;
        cur_task.stamp = ros::Time::now();
    }
}
 
// 左右点云数据的回调函数：获取左右障碍物的距离
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    obs_dis=999;
    PointCloud cloud;
    pcl::fromROSMsg(*c_msg, cloud);

    int filter_num=10;
    nh->getParam("filter_num", filter_num);
    if(filter_num>1 && cloud.size()>1)  cloud=Cloud_FastFilter(cloud, filter_num, 0.2);

    if(cloud.size()<10)  { return; }

    PointType cloud_min, cloud_max;
    pcl::getMinMax3D(cloud, cloud_min, cloud_max);
    if(dir=="left")  
    {
        obs_dis=fabs(cloud_min.y)-0.5*agv_width;
    }
    else  obs_dis=fabs(cloud_max.y)-0.5*agv_width;
}

//剩余路径回调函数
void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length = msg->data;
}

//根据距离的长短得到速度大小
float GetVelByDistance(float max_d, float max_vel, float min_d, float min_vel, float cur_d)
{
    float res;
    float Vmid=1; 
    float k=(max_vel-min_vel)/(max_d-min_d);
    float b=0.05-k*min_d;
    float sc=(Vmid-b)/k;
    float a=(max_vel-Vmid)/(pow(max_d,2.5)-pow(sc,2.5));
    float b2=max_vel-a*pow(max_d,2.5);

    if(cur_d>=max_d)  res=max_vel;
    else if (cur_d>=sc)  res=a*pow(cur_d,2.5)+b2;
    else if (cur_d>=min_d)  res=k*cur_d+b;
    else  res=min_vel;

    return res;
}

//速度限制
void GetSpeedLimit(float obs_dis, float &speedlimit)
{
    float stop_dis=0.6;
    if(cur_task.final_path && remain_path_length<8) 
    {
        if(cur_task.cmd=="charge task")  stop_dis=0.15;
        else  stop_dis=0.2;
    }
    else stop_dis=0.6;

    float speed=GetVelByDistance(15,3,stop_dis,0,obs_dis);
    float dspeed=speed-speedlimit;
    if(dspeed>0.01) dspeed=0.01;
    // printf("dsp=%.2f\n",dspeed);
    // else if(dspeed>-0.4 && dspeed<0) dspeed=-0.4;
    speedlimit+=dspeed;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_calculation_leftright");
    nh=new ros::NodeHandle("~");
    
    nh->getParam("dir", dir);

    TNodeCheck *nodecheck=new TNodeCheck(nh, "node_rate");

    ros::Subscriber cloud_sub;
    if(dir=="left")  cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_left_cloud", 10, &PointCloudCallback);
    else cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_right_cloud", 10, &PointCloudCallback); 
    ros::Subscriber task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, TaskCallback);
    ros::Subscriber remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &RemainPathCallback);

    ros::Publisher pub_obs_dis = nh->advertise<std_msgs::Float32MultiArray>("obs_dis", 10);
    ros::Publisher pub_obs_speedlimit = nh->advertise<std_msgs::Float32MultiArray>("obs_speedlimit", 10);

    TDataFilter f_obs_dis(10);

    ros::Rate looprate(20);
    while (ros::ok())
    {
        nodecheck->Find("node_rate")->Beat();
 
        obs_dis=f_obs_dis.GetValue(obs_dis);
        float speedlimit=999;
        GetSpeedLimit(obs_dis, speedlimit);

        std_msgs::Float32MultiArray msg;
        msg.data.push_back(obs_dis);
        pub_obs_dis.publish(msg);
        std_msgs::Float32MultiArray speedmsgs;
        speedmsgs.data.push_back(speedlimit);
        pub_obs_speedlimit.publish(speedmsgs);

        ros::spinOnce();
        looprate.sleep();
    } 
    
    ros::shutdown();
    return 0;
}