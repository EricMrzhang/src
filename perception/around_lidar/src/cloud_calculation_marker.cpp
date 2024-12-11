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
#include <mqtt_comm/task.h>

#include "cloud_preprocess.h"
#include <vector>
#include <algorithm>

using namespace std;
//发布marker位姿、
ros::NodeHandle *nh;
float agv_width=3, agv_length=5.99, agv_inside_width=2.105;

ros::Publisher pub_agv_pos;
mqtt_comm::task cur_task;
float remainpath=0;
geometry_msgs::PoseStamped pose_target;

string dir="front";
string move_dir="";

//marker比较
bool markercmp(visualization_msgs::Marker m1,visualization_msgs::Marker m2)
{
    return m1.pose.position.x<m2.pose.position.x;
}

//发布marker位姿
void PubMarkerPose(PointCloud cloud)
{
    // printf("size=%d\n",cloud.size());
    if(cloud.size()<2)  return;
    // printf("cur_task.subcmd=%s\n",cur_task.subcmd.c_str());
    if(cur_task.cmd!="release task" || cur_task.subcmd!="2" || !cur_task.final_path || remainpath>1) return;

    float cluster_dis=0.1;  //聚类距离
    ////点云聚类:欧几里得距离的聚类/KD-tree搜索
    visualization_msgs::MarkerArray ma=Clould_Cluster(cloud, cluster_dis, 2, 1000);
    // printf("n=%d size=%d\n", ma.markers.size(), cloud.size());
    if(ma.markers.size()<1)  return;

    // ROS_INFO("dt1=%.5f", ros::Time::now().toSec()-t0);
     //  距离降序排列
    sort(ma.markers.begin(), ma.markers.end(), markercmp); 
     //marker最小的位姿
    float marker_x=ma.markers[0].pose.position.x;
    float marker_y=ma.markers[0].pose.position.y;
    float width=ma.markers[0].scale.y;
    // ROS_INFO("width= %.2f y=%.2f",width, marker_y);
    //  加入对标记点几何约束
    if(fabs(marker_y)>2.5 || fabs(marker_y)<1.5 || width>0.15)  return;  

    geometry_msgs::PoseStamped target_base;
    pose_target.header.stamp=ros::Time::now();
    //将 pose_target从源坐标系 "base_link" 转换到目标坐标系 target_base，
    transformPose("base_link",pose_target,target_base,"CCC");

    float marker_stop_dis=0.16;  //  标记物停车距离
    //dx
    float dx=target_base.pose.position.x-marker_x+0.5*agv_length;
    // ROS_INFO("%.2f",dx);
    if(fabs(dx)<1)   // 是否在一定范围内
    {
        // target_base.pose.position.x=marker_x;
        //根据偏航角添加位置点
        target_base=GetExtendPoseByPose(target_base, -dx-marker_stop_dis);
         //marker_dis 
        float marker_dis = fabs(marker_x)-0.5*agv_length;
        // 设置速度
        if(marker_dis<marker_stop_dis+0.5)  target_base.pose.position.z=0.05;
        else if(marker_dis<marker_stop_dis+1.5)  target_base.pose.position.z=0.2;
        else  target_base.pose.position.z=0.3;

        // ROS_INFO("%.2f",marker_dis);
        //发布目标位姿
        pub_agv_pos.publish(target_base);
    }
}

// 前侧带反光膜标记点云的回调函数，计算中心距离和前后距离
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &c_msg)
{
    PointCloud cloud;
    pcl::fromROSMsg(*c_msg, cloud);

    // printf("%s | %s\n",move_dir.c_str(), dir.c_str());
    if(move_dir==dir) PubMarkerPose(cloud);
}
//剩余路径回调函数
void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remainpath=msg->data;
}
//移动方向回调函数
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

        if(msg->subcmd=="2" && cur_task.final_path && cur_task.path.size())
        {
            pose_target.header.frame_id="map";
            //最后一个路径点
            pose_target.pose.position.x=(cur_task.path.end()-1)->pointX;
            pose_target.pose.position.y=(cur_task.path.end()-1)->pointY;
            pose_target.pose.position.z=0;

            float angle=(cur_task.path.end()-1)->pointHA;
            //将一个给定的偏航角（yaw angle）转换为四元数（quaternion），
            //并将该四元数设置为 pose_target 对象的姿态
            pose_target.pose.orientation=tf::createQuaternionMsgFromYaw(angle/180*M_PI);
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
    ros::Subscriber move_dir_sub = nh->subscribe<std_msgs::String>("/move_dir", 10, MoveDirCallback);
    
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