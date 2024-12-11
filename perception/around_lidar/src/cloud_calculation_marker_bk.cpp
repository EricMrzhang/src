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
#include <mqtt_comm/task.h>

#include "cloud_preprocess.h"
#include <vector>
#include <algorithm>

using namespace std;
////发布marker位姿、根据目标位置获取偏移量、
//立体停车位坐标、偏移量结构体定义和初始化

ros::NodeHandle *nh;
float agv_width=3, agv_length=5.99, agv_inside_width=2.105;

ros::Publisher pub_agv_pos;
mqtt_comm::task cur_task;
float remainpath=0;

string dir="front";
string move_dir="";
float marker_bias=0.165;

//立体停车位坐标、偏移量
struct LT_Pos
{
    float x, y;   //  停车位坐标
    float bias;   //  标记偏移量
};
vector<LT_Pos> LT_Pos_Array;

// 立体停车位初始化：1-1,1-2,1-3
void LT_Init()   //  立体停车位初始化
{
    LT_Pos ltp;
    
    // 1-1
    ltp.x=120,  ltp.y=89,  ltp.bias=0.165;
    LT_Pos_Array.push_back(ltp);
    // 1-2
    ltp.x=150,  ltp.y=89,  ltp.bias=-0.125;
    LT_Pos_Array.push_back(ltp);
    // 1-3
    ltp.x=-92.22,  ltp.y=297.80,  ltp.bias=0.065;
    LT_Pos_Array.push_back(ltp);

    //  继续添加
}

//  根据目标位置获取偏离量
float GetMarkerBias(float x, float y)  
{
    float res=0;
    for(auto it:LT_Pos_Array)
    {
        float ds=sqrt((x-it.x)*(x-it.x)+(y-it.y)*(y-it.y));
        if(ds<2)    //  根据位置判断停车位
        {
            res=it.bias;
            break;
        }
    }
    // ROS_INFO("bias=%.5f",res);
    return res;
}

//marker位置比较
bool markercmp(visualization_msgs::Marker m1,visualization_msgs::Marker m2)
{
    float l1=pow(m1.pose.position.x,2)+pow(m1.pose.position.y,2);
    float l2=pow(m2.pose.position.x,2)+pow(m2.pose.position.y,2);
    return l1<l2;
}

////发布marker位姿(重要)
void PubMarkerPose(PointCloud cloud)
{
    // printf("size=%d\n",cloud.size());

    if(cloud.size()<2)  return;
    // printf("cur_task.subcmd=%s\n",cur_task.subcmd.c_str());
    if(cur_task.cmd!="release task" || cur_task.subcmd!="2" || remainpath>10) return;

    // 根据点云数量自适应改变滤波范围与聚类参数
    float cluster_dis=0.5;
    float height=0.4;
    // if(cloud.size()>400)  height=0.05, cluster_dis=0.05;
    // else if(cloud.size()>200)  height=0.2,  cluster_dis=0.1;

    // cloud=Cloud_PassThrough(cloud, 0, 0, 0, 0, -height, height);
    //点云聚类:欧几里得距离的聚类/KD-tree搜索
    visualization_msgs::MarkerArray ma=Clould_Cluster(cloud, cluster_dis, 2,1000);
    // printf("n=%d size=%d\n", ma.markers.size(), cloud.size());
    if(ma.markers.size()<2)  return;

    // ROS_INFO("dt1=%.5f", ros::Time::now().toSec()-t0);

    sort(ma.markers.begin(), ma.markers.end(), markercmp);  //  距离降序排列
    // marker最小、最小其次的位姿
    float x1=ma.markers[0].pose.position.x;
    float y1=ma.markers[0].pose.position.y;
    float x2=ma.markers[1].pose.position.x;
    float y2=ma.markers[1].pose.position.y;
    // printf("(%.2f,%.2f),(%.2f,%.2f)\n",x1,y1,x2,y2);
    //
    float angle=atan2(y1-y2,x1-x2)*180/M_PI;
    if(fabs(angle+90)<30)  angle+=90;
    else angle-=90;
    float mid_x=(x1+x2)*0.5;
    float mid_y=(y1+y2)*0.5+marker_bias;
    float len=sqrt(pow(x1-x2,2)+pow(y1-y2,2)); 
    // printf("c=%.1f len=%.2f, angle=%.1f\n",y1*y2, len,angle);
    // printf("n=%d id_x=%.2f ,mid_y=%.2f angle=%.1f\n", ma.markers.size(),mid_x,mid_y, angle);
    //  加入对标记点几何约束
    if((y1*y2>0 || len<1.5 || len>4 || fabs(angle)>10 || mid_x>8))  return;   //  加入对标记点几何约束

    geometry_msgs::PoseStamped pose_marker;
    pose_marker.header.frame_id="base_link";
    pose_marker.header.stamp=ros::Time::now();
    pose_marker.pose.orientation=tf::createQuaternionMsgFromYaw(angle/180*M_PI);
    pose_marker.pose.position.x=mid_x;
    pose_marker.pose.position.y=mid_y;
    //  标记物停车距离        
    float marker_stop_dis=0.2;  
    float marker_dis = fabs(mid_x)-0.5*agv_length;
    // pose.pose.position.z=GetVelByDistance(10,0.5,marker_stop_dis,0.1,marker_dis);
    
    if(marker_dis<marker_stop_dis+0.5)  pose_marker.pose.position.z=0.06;
    else if(marker_dis<marker_stop_dis+1.5)  pose_marker.pose.position.z=0.2;
    else  pose_marker.pose.position.z=0.3;
    // pose.pose.position.z=GetVelByDistance(10,0.5,marker_stop_dis,0.1,marker_dis);
    // printf("speed=%.2f marker_dis=%.2f\n",pose.pose.position.z,marker_dis);
    //GetExtendPoseByPose根据偏航角添加位置点
    if(move_dir=="back")  pose_marker=GetExtendPoseByPose(pose_marker, marker_stop_dis+0.5*agv_length);
    else if(move_dir=="front") pose_marker=GetExtendPoseByPose(pose_marker, -marker_stop_dis-0.5*agv_length);
    // printf("mid_x=%.2f ,mid_y=%.2f,marker_dis=%.2f ,speed=%.2f,mov_dir=%s\n",mid_x,mid_y,marker_dis,pose.pose.position.z,mov_dir.c_str());
    // printf("speed=%.2f marker_dis=%.2f\n",pose.pose.position.z,marker_dis);
    // pub_agv_pos.publish(pose);

    // 主线规划的点
    geometry_msgs::PoseStamped pose_base,pose_map;
    pose_map.header.frame_id="map";
    pose_map.header.stamp = ros::Time::now();
    float anglex=(cur_task.path.end()-1)->pointHA;
    pose_map.pose.orientation=tf::createQuaternionMsgFromYaw(anglex/180*M_PI);
    pose_map.pose.position.x = (cur_task.path.end()-1)->pointX;
    pose_map.pose.position.y =(cur_task.path.end()-1)->pointY;
    pose_map.pose.position.z=0; 
    transformPose("base_link",pose_map,pose_base,"CCC");
    
    geometry_msgs::PoseStamped pose_msg=pose_base;
     // 用标记物作为前向距离
    pose_msg.pose.position.x=pose_marker.pose.position.x; 
    pose_msg.pose.position.z=pose_marker.pose.position.z;  // 运动速度值
    
    // pose_msg.header = pose_base.header;
    // pose_msg.pose.orientation=pose_base.pose.orientation;
    // pose_msg.pose.position.x=pose_marker.pose.position.x;  // 用标记物作为前向距离
    // pose_msg.pose.position.y=pose_base.pose.position.y;
    // pose_msg.pose.position.z=pose_marker.pose.position.z;   
    
    geometry_msgs::PoseStamped pose_bit_map;
    transformPose("map",pose_marker,pose_bit_map,"CCC");
    if(P2P(pose_bit_map.pose.position,pose_map.pose.position)<1) pub_agv_pos.publish(pose_msg);
}

// 前侧带反光膜标记点云的回调函数，计算中心距离和前后距离
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    PointCloud cloud;
    pcl::fromROSMsg(*c_msg, cloud);

    // printf("size=%d\n",cloud.size());
    if(move_dir==dir) PubMarkerPose(cloud);
}
//剩余路径回调函数
void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remainpath=msg->data;
}
//运动方向回调函数
void MoveDirCallback(const std_msgs::String::ConstPtr &msg)
{
    move_dir=msg->data;
}

//  接收任务指令
void TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    // ROS_ERROR("%s\n", msg->cmd.c_str());
    if (msg->cmd.find("task") != string::npos)
    {
        cur_task = *msg;
        cur_task.stamp = ros::Time::now();

        if(msg->subcmd=="2" && cur_task.path.size())
        {
            float x=(cur_task.path.end()-1)->pointX;
            float y=(cur_task.path.end()-1)->pointY;
            marker_bias=GetMarkerBias(x,y);
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_calculation_marker");
    nh=new ros::NodeHandle("~");
    
    TNodeCheck *nodecheck=new TNodeCheck(nh, "node_rate");

    nh->getParam("dir", dir);

    ros::Subscriber cloud_sub;
    if(dir=="front")  cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_frontmarker_cloud", 10, &PointCloudCallback);
    else  cloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/cloud_segmentation/agv_backmarker_cloud", 10, &PointCloudCallback);
    ros::Subscriber task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, TaskCallback);
    ros::Subscriber remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, RemainPathCallback);
    
    pub_agv_pos = nh->advertise<geometry_msgs::PoseStamped>("agv_pose",10);

    ros::Rate looprate(20);
    while (ros::ok())
    {
        nodecheck->Find("node_rate")->Beat();

        ros::spinOnce();
        looprate.sleep();
    } 
    
    ros::shutdown();
    return 0;
}