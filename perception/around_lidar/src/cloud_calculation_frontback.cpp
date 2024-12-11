//本节点输入为前后左右四路激光点云,输出前后左右障碍物最近距离和前后纠偏误差

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <ros/spinner.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
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
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include "pointcloud_type.h"
#include "common/public.h"
#include <mqtt_comm/task.h>

#include "cloud_preprocess.h"
#include <vector>
#include <algorithm>

//本节点输入为前后两路路激光点云,输出前后障碍物最近距离和前后纠偏误差

using namespace std;

ros::NodeHandle *nh;
float agv_width=3, agv_length=5.99, agv_inside_width=2.105; 

float err=999, car_dis=999, middle_obs_dis=999, side_obs_dis=999, car_distance=999;

string dir="front";
string move_dir="";
nav_msgs::Path localpath;

ros::Publisher pub_cloud_checkerr, pub_cloud_side;
mqtt_comm::task cur_task;
bool agv_in_flag=false;
TMinBoundingBox *mbb;

float remain_path_length = 0;
//  接收任务指令 回调函数
void TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    // ROS_ERROR("%s\n", msg->cmd.c_str());
    //常用于指示某个操作（如查找、搜索等）未能在字符串中找到目标
    //子串或字符时的位置。string::npos 的值通常是 -1 的无符号整数表示
    //判读是否存在控制指令
    if (msg->cmd.find("task") != string::npos)
    {
        cur_task = *msg;
        cur_task.stamp = ros::Time::now();
    }
}

//agv_in_flsg
//将消息的第一个数据赋值给agv_in_flag
void AgvInCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (msg->data.size() == 2)
    {        
        agv_in_flag = msg->data[0]; // agv_in
    }
}

//比较两点的y轴值的大小
bool mycmp(visualization_msgs::Marker m1,visualization_msgs::Marker m2)
{
    return fabs(m1.pose.position.y)<fabs(m2.pose.position.y);
}


// 根据前方或后方点云计算横向偏移量,作为纠偏反馈量
float CheckErr(PointCloud cloud)   
{
    float err=999;
    static TDataFilter filter_err(5);

    float speedlimit=999;
    nh->getParam("/mi_stop/speedlimit", speedlimit);
    bool Check_Err_Enable=(cur_task.cmd=="pick task" && !agv_in_flag) || (cur_task.cmd=="release task" && cur_task.subcmd=="1" && speedlimit>0.5);

    if(cloud.size()>10 && Check_Err_Enable) 
    {
        // ROS_INFO("CHECK!");
        geometry_msgs::Point s;
        s.x=s.y=s.z=0.1;
        cloud=Cloud_DownSampling(cloud, s);
        visualization_msgs::MarkerArray ma=Clould_Cluster(cloud,0.16,5,10000);
        // // ROS_INFO("1=n=%d", ma.markers.size());

        visualization_msgs::MarkerArray ma_tmp=ma;
        ma_tmp.markers.clear();
        for(auto it:ma.markers)
            if(it.scale.y>1 && fabs(it.pose.position.y)<0.8)  ma_tmp.markers.push_back(it);                
        ma=ma_tmp;

        if(ma.markers.size()>0)
        {
            sort(ma.markers.begin(), ma.markers.end(), mycmp);
            err=filter_err.GetValue(ma.markers.begin()->pose.position.y);
            // ROS_INFO("n=%d err=%.2f", ma.markers.size(), err);

            float x_min, y_min, x_max, y_max;
            x_min=ma.markers.begin()->pose.position.x-ma.markers.begin()->scale.x*0.5;
            x_max=ma.markers.begin()->pose.position.x+ma.markers.begin()->scale.x*0.5;
            y_min=ma.markers.begin()->pose.position.y-ma.markers.begin()->scale.y*0.5;
            y_max=ma.markers.begin()->pose.position.y+ma.markers.begin()->scale.y*0.5;
            cloud=Cloud_PassThrough(cloud, x_min, x_max, y_min, y_max, 0, 0);

            sensor_msgs::PointCloud2 msgx;
            pcl::toROSMsg(cloud, msgx);
            msgx.header.frame_id = cloud.header.frame_id;
            msgx.header.stamp=ros::Time::now();
            pub_cloud_checkerr.publish(msgx);

            // ROS_INFO("n=%d %.2f %.2f", ma.markers.size(),ma.markers[0].pose.position.x,ma.markers[0].pose.position.y);
            //  ROS_INFO("n=%d x=%.2f y=%.2f, width=%.2f", ma.markers.size(), ma.markers.begin()->pose.position.x,ma.markers.begin()->pose.position.y, ma.markers.begin()->scale.y);
        }
    }
    return err;   
}

//点云滤波
PointCloud CloudRingFilter(PointCloud cloud)
{
    PointCloud cloud_filter;
    cloud_filter.header=cloud.header;
    //如果点不在环数4到9之间或强度大于2，则添加到cloud_filter
    for(auto it:cloud)
        if(!(it.ring>=4 && it.ring<=9 && it.intensity<=2))  cloud_filter.push_back(it);
    return cloud_filter;   
}

//点云滤波、检测周围障碍物的距离
void CheckMiddleSideCloud(PointCloud cloud, float &middle_obs_dis, float &side_obs_dis)
{
    float inside_width=0.02, outside_width=0.05;
    ////根据x,y,z极值差大小，筛选点云
    PointCloud middle_cloud = Cloud_PassThrough(cloud, 0, 0, -agv_inside_width * 0.5, agv_inside_width * 0.5, 0, 0);
     //点云滤波
    middle_cloud=CloudRingFilter(middle_cloud);
    //根据x,y,z极值差大小，筛选点云
    PointCloud side_cloud1 = Cloud_PassThrough(cloud, 0, 0, -agv_width*0.5-outside_width, -agv_inside_width * 0.5+inside_width, 0, 0);
    //点云滤波
    side_cloud1=CloudRingFilter(side_cloud1);
    ////根据x,y,z极值差大小，筛选点云
    PointCloud side_cloud2 = Cloud_PassThrough(cloud, 0, 0, agv_inside_width * 0.5-inside_width, agv_width * 0.5+outside_width, 0, 0);
    //点云滤波
    side_cloud2=CloudRingFilter(side_cloud2);

    float side_obs_dis1=999, side_obs_dis2=999;
    PointType cloud_min, cloud_max;
    
    //计算中间离障碍物的距离
    if(middle_cloud.size()>0)
    {
        //用于计算点云中所有点在X、Y、Z三个维度上的最小值和最大值的函数。
        //这个函数通常被用来获取点云的空间边界框
        pcl::getMinMax3D(middle_cloud, cloud_min, cloud_max);
        if(dir=="front")  middle_obs_dis = fabs(cloud_min.x) - agv_length * 0.5;
        else  middle_obs_dis = fabs(cloud_max.x) - agv_length * 0.5;
    }
    
    //计算传感器1检测到离障碍物的距离
    if(side_cloud1.size()>0)
    {
        pcl::getMinMax3D(side_cloud1, cloud_min, cloud_max);
        if(dir=="front")  side_obs_dis1 = fabs(cloud_min.x) - agv_length * 0.5;
        else  side_obs_dis1 = fabs(cloud_max.x) - agv_length * 0.5;
    }
    //计算传感器2检测到离障碍物的距离
    if(side_cloud2.size()>0)
    {
        pcl::getMinMax3D(side_cloud2, cloud_min, cloud_max);
        if(dir=="front")  side_obs_dis2 = fabs(cloud_min.x) - agv_length * 0.5;
        else  side_obs_dis2 = fabs(cloud_max.x) - agv_length * 0.5;
    }
    //计算side离障碍物的距离
    side_obs_dis = min(side_obs_dis1, side_obs_dis2);

    // if(dir=="back")  ROS_INFO("side1=%.1f side2=%.1f", side_obs_dis1,side_obs_dis2);
    PointCloud clouds=side_cloud1+side_cloud2;
    sensor_msgs::PointCloud2 msgx;
    pcl::toROSMsg(clouds, msgx);
    msgx.header.frame_id = clouds.header.frame_id;
    msgx.header.stamp=ros::Time::now();
    //发布 side1、side2 的点云信息
    pub_cloud_side.publish(msgx);
}

// 判断点是否在路径上，len为最大允许偏离距离
bool PointInLocalPath(geometry_msgs::PointStamped point, float len)
{
    bool res=false;

    if(localpath.poses.size()>0)
    {
        float dis;
        ////  PointDistoPath(获得路径上最近点ID
        PointDistoPath(localpath, point, dis);
        if(dis<len)  res=true;  
        // ROS_INFO("dis=%.2f",dis);
    }

    return res;
}

// 前方点云数据的回调函数：获取前方障碍物的距离，根据前方对接商品车纠偏
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    PointCloud cloud;
    // 使用pcl::fromROSMsg将ROS消息转换为PCL点云  
    pcl::fromROSMsg(*c_msg, cloud);

    int filter_num=0;
    // nh->getParam("filter_num", filter_num);
    //Cloud_FastFilter点云快速聚类
    if(filter_num>1 && cloud.size()>1)  cloud=Cloud_FastFilter(cloud, filter_num, 0.2);

    err=car_dis=middle_obs_dis=side_obs_dis=999;
    if(cloud.size()>10)  
    {
        //点云滤波、检测周围障碍物的距离
        CheckMiddleSideCloud(cloud, middle_obs_dis, side_obs_dis);

        PointType cloud_min, cloud_max;
        //计算点云（PointCloud）中所有点在三维空间中的最小和最大坐标值。
        pcl::getMinMax3D(cloud, cloud_min, cloud_max);

        geometry_msgs::PointStamped center_point;
        center_point.header.frame_id="base_link";
        center_point.header.stamp=ros::Time::now();
        if(dir=="front")  center_point.point.x=cloud_min.x;
        else if(dir=="back")  center_point.point.x=cloud_max.x;
        center_point.point.y=(cloud_min.y+cloud_max.y)*0.5; 
        
        float width = cloud_max.y - cloud_min.y;
        float len = cloud_max.x - cloud_min.x;
        // ROS_INFO("width=%.2f move_dir=%s PointInLocalPath %d", width,move_dir.c_str(),PointInLocalPath(center_point,1));
        if(width>1 && len>1.2 && move_dir==dir && PointInLocalPath(center_point,3)) // 对车体外形做约束
        {  
            //纠偏
            err=CheckErr(cloud);  

            car_dis=fabs(center_point.point.x)-agv_length*0.5;
            // ROS_INFO("car_dis_front  %.2f", car_dis);
        }
    }
}
//剩余路径回调函数
void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length = msg->data;
}

//运动 方向回调函数
void MoveDirCallback(const std_msgs::String::ConstPtr &msg)
{
    move_dir=msg->data;
}
// 接收局部路径 获取预瞄点
void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg) 
{
    localpath = *msg;
}
//搬运车距离
void CarDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) 
{
    car_distance = msg->data[0];
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
    
    int lane_type;
    nh->getParam("/local_path_plan/lane_type", lane_type);
    float final_path_len=10, final_car_dis=6;
    bool field_flag = lane_type==23 || lane_type==24 || lane_type==25;
    bool is_final_path=(remain_path_length<final_path_len || (car_distance<final_car_dis && field_flag)) && cur_task.final_path;

    if(is_final_path) stop_dis=0.2;
    else stop_dis=0.6;
    //根据距离的长短得到速度大小
    float speed=GetVelByDistance(15,3,stop_dis,0,obs_dis);  
    float dspeed=speed-speedlimit;  //
    if(dspeed>0.01) dspeed=0.01;
    // printf("dsp=%.2f\n",dspeed);
    // else if(dspeed>-0.4 && dspeed<0) dspeed=-0.4;
    speedlimit+=dspeed;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_calculation_frontback");
    nh=new ros::NodeHandle("~");
    
    TNodeCheck *nodecheck=new TNodeCheck(nh, "node_rate");

    nh->getParam("dir", dir);
    // printf("move=%s\n",dir.c_str());
    
    ros::Subscriber cloud_sub;
    //点云选择在前或后
    if(dir=="front")  cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_front_cloud", 10, &PointCloudCallback);
    else cloud_sub=nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_back_cloud", 10, &PointCloudCallback); 
    //订阅话题
    ros::Subscriber task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, TaskCallback);
    ros::Subscriber agv_in_sub = nh->subscribe<std_msgs::Int32MultiArray>("/cloud_calculation_inside/agv_in_flag", 10, &AgvInCallback);
    ros::Subscriber remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &RemainPathCallback);
    ros::Subscriber move_dir_sub = nh->subscribe<std_msgs::String>("/move_dir", 10, &MoveDirCallback);
    ros::Subscriber localpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, &LocalPathCallback);
    ros::Subscriber car_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/pathtrack/car_in_out_dis",10, &CarDisCallback);
     //发布话题
    ros::Publisher pub_obs_dis = nh->advertise<std_msgs::Float32MultiArray>("obs_dis", 10);
    ros::Publisher pub_obs_speedlimit = nh->advertise<std_msgs::Float32MultiArray>("obs_speedlimit", 10);
    ros::Publisher pub_agv_err = nh->advertise<std_msgs::Float32MultiArray>("agv_lidar_err", 10);
    
    pub_cloud_checkerr = nh->advertise<sensor_msgs::PointCloud2>("cloud_checkerr", 10);
    pub_cloud_side = nh->advertise<sensor_msgs::PointCloud2>("cloud_side", 10);

    TDataFilter f_middle_obs_dis(5), f_side_obs_dis(5), f_car_dis(5);

    ros::Rate looprate(20);
    while (ros::ok())
    {
        nodecheck->Find("node_rate")->Beat();

        middle_obs_dis=f_middle_obs_dis.GetValue(middle_obs_dis);
        side_obs_dis=f_side_obs_dis.GetValue(side_obs_dis);
        car_dis=f_car_dis.GetValue(car_dis);

        std_msgs::Float32MultiArray dis_msg;
        dis_msg.data.push_back(middle_obs_dis);
        dis_msg.data.push_back(side_obs_dis);
        dis_msg.data.push_back(car_dis);
        pub_obs_dis.publish(dis_msg);   //发布dis_msg消息

        float middle_speedlimit=999, side_speedlimit=999;
        //根据
        GetSpeedLimit(middle_obs_dis, middle_speedlimit);
        GetSpeedLimit(side_obs_dis, side_speedlimit);

        // if(mov_dir=="front" && dir=="front")  ROS_INFO("dis=%.2f vel=%.2f", side_obs_dis, side_speedlimit);
        
        std_msgs::Float32MultiArray speed_msg;
        speed_msg.data.push_back(middle_speedlimit);
        speed_msg.data.push_back(side_speedlimit);
        pub_obs_speedlimit.publish(speed_msg);

        std_msgs::Float32MultiArray err_msg;
        err_msg.data.push_back(err);
        pub_agv_err.publish(err_msg);  //发布纠偏误差

        ros::spinOnce();
        looprate.sleep();
    } 
    
    ros::shutdown();
    return 0;
}