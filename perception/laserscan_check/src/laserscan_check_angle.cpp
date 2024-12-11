#define PCL_NO_PRECOMPILE

#include <common/public.h>

#include <Eigen/Core>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
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
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <data_comm/car_state.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include "cloud_preprocess.h"

// 本程序输入为线激光雷达点云,经过三维点云数据转换,范围滤波和聚类处理后,获得轮胎扫描线的长度,中心位置和距离
// BUG 速度1Hz，去掉欧式聚类

using namespace std;
data_comm::car_state cur_carstate;
class CLaserscan_Check
{
private:
    // 0~3内部线激光雷达
    ros::NodeHandle *nh_local;
    ros::Subscriber laserscan_sub[4], remainpath_sub, passedpath_sub,carstate_sub;
    ros::Publisher wheel_err_pub, cloud_pub[4], distancemarker_pub;

    TNodeCheck *nodecheck;
    // float remain_path_len = 0, passed_path_len = 0;
    float box_x_min = 0.05, box_x_max = 0.8 ;
    float box_y_min = -0.5, box_y_max = 3.5;
    geometry_msgs::Point center_point[4];
    // data_comm::car_state cur_carstate;
    bool laser_single_enable=true;
public:

    CLaserscan_Check(ros::NodeHandle* node);
    void LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void CarStateCallback(const data_comm::car_state::ConstPtr &msg);

    // void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    // void PassedPathCallback(const std_msgs::Float64::ConstPtr &msg);

    void CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg);
    void WheelErrPublish();
};

//激光扫描检查：激光雷达类型、车轮误差 、扫描频率
CLaserscan_Check::CLaserscan_Check(ros::NodeHandle* node): nh_local(node)
{
    string scan_lidar_type;
    nh_local->getParam("/scanlidar",scan_lidar_type);
    //搬运车状态
    carstate_sub = nh_local->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &CLaserscan_Check::CarStateCallback, this);
   
    //判断激光雷达类型是否是C200
    if(scan_lidar_type=="c200")
    {
        laserscan_sub[0] = nh_local->subscribe<sensor_msgs::LaserScan>("/c200_lidar_node1/scan", 10, &CLaserscan_Check::LaserScanCallback1, this);
        laserscan_sub[1] = nh_local->subscribe<sensor_msgs::LaserScan>("/c200_lidar_node2/scan", 10, &CLaserscan_Check::LaserScanCallback2, this);
        laserscan_sub[2] = nh_local->subscribe<sensor_msgs::LaserScan>("/c200_lidar_node3/scan", 10, &CLaserscan_Check::LaserScanCallback3, this);
        laserscan_sub[3] = nh_local->subscribe<sensor_msgs::LaserScan>("/c200_lidar_node4/scan", 10, &CLaserscan_Check::LaserScanCallback4, this);
    }
    else 
    {
        laserscan_sub[0] = nh_local->subscribe<sensor_msgs::LaserScan>("/olelidar1/scan", 10, &CLaserscan_Check::LaserScanCallback1, this);
        laserscan_sub[1] = nh_local->subscribe<sensor_msgs::LaserScan>("/olelidar2/scan", 10, &CLaserscan_Check::LaserScanCallback2, this);
        laserscan_sub[2] = nh_local->subscribe<sensor_msgs::LaserScan>("/olelidar3/scan", 10, &CLaserscan_Check::LaserScanCallback3, this);
        laserscan_sub[3] = nh_local->subscribe<sensor_msgs::LaserScan>("/olelidar4/scan", 10, &CLaserscan_Check::LaserScanCallback4, this);
    }

    // remainpath_sub = nh_local->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &CLaserscan_Check::RemainPathCallback, this);
    // passedpath_sub = nh_local->subscribe<std_msgs::Float64>("/local_path_plan/passedpath", 10, &CLaserscan_Check::PassedPathCallback, this);
    //发布点云
    cloud_pub[0] = nh_local->advertise<sensor_msgs::PointCloud2>("cloud1_filter", 10);
    cloud_pub[1] = nh_local->advertise<sensor_msgs::PointCloud2>("cloud2_filter", 10);
    cloud_pub[2] = nh_local->advertise<sensor_msgs::PointCloud2>("cloud3_filter", 10);
    cloud_pub[3] = nh_local->advertise<sensor_msgs::PointCloud2>("cloud4_filter", 10);
    
    //车轮误差发布
    wheel_err_pub = nh_local->advertise<std_msgs::Float32MultiArray>("wheel_err", 10);

    // nodecheck = new TNodeCheck(nh_local, "node_rate");
    //scan1_rate 激光扫描频率
    nodecheck = new TNodeCheck(nh_local, "node_rate scan1_rate scan2_rate scan3_rate scan4_rate");
    // nodecheck->Find("node_rate")->SetLimit(10);
}

//搬运车状态回调函数
void CLaserscan_Check::CarStateCallback(const data_comm::car_state::ConstPtr &msg) //  接收车体状态信息
{
    cur_carstate = *msg;
    // ROS_INFO("%d ", cur_carstate.holdcar);
}

//比较两个marker在y坐标系上的大小
bool ma_cmp(visualization_msgs::Marker m1,visualization_msgs::Marker m2)
{
    return fabs(m1.pose.position.y)<fabs(m2.pose.position.y);
}

//点云处理？？？
void CLaserscan_Check::CloudProcess(int id, const sensor_msgs::LaserScan::ConstPtr &c_msg) // pcl::PointCloud<pcl::PointXYZI> cloud, geometry_msgs::Point &point)
{   
    // pcl::PointCloud<pcl::PointXYZI>是一个用于存储点云数据的模板类，其中每个点都包含XYZ坐标（位置）和强度（Intensity）信息。
    pcl::PointCloud<pcl::PointXYZI> cloud;
    //从激光扫描到点云的转换
    laser_geometry::LaserProjection projection;
    //点云数据的消息类型，它被广泛用于在ROS节点之间传输点云数据
    sensor_msgs::PointCloud2 cloud_msg;
    //
    projection.projectLaser(*c_msg, cloud_msg);
    pcl::fromROSMsg(cloud_msg, cloud);

    float x=0;
    for(auto it:cloud) x+=fabs(it.x);
    if(x>0.1)  
    {
        char str[100];
        sprintf(str,"scan%d_rate", id+1);
        nodecheck->Find(str)->Beat();
    }

    if(cur_carstate.holdcar==1)  return;

    center_point[id].x = -1;
    center_point[id].y = -1; //  not reach
    center_point[id].z = 0;

    // 检测线雷达外侧
    if(id==0 || id==1)  cloud = Cloud_PassThrough(cloud, box_x_min, box_x_max, -box_y_max, -box_y_min, 0, 0);
    else  cloud = Cloud_PassThrough(cloud, box_x_min, box_x_max, box_y_min, box_y_max, 0, 0);
    // cloud = Cloud_RadiusFilter(cloud, 0.2, 3);
        
    visualization_msgs::MarkerArray ma = Clould_Cluster(cloud, 0.12, 2, 10000), ma_tmp;  //

    // if(id==1) ROS_INFO("SIZE=%d",ma.markers.size());
    for(auto it:ma.markers)  //  过滤掉远离边沿点云
        // if(it.pose.position.x<0.6 && it.scale.x<0.4 && it.scale.y<0.6)  
        if(it.pose.position.x<0.6 && it.scale.x<0.4 && it.scale.y<0.8)  
            ma_tmp.markers.push_back(it);
        // if(it.pose.position.x<0.6 && it.scale.x<0.8 && it.scale.y<0.8)  ma_tmp.markers.push_back(it);
    ma.markers=ma_tmp.markers;    
    
    if(ma.markers.size()>0)
    {
        sort(ma.markers.begin(), ma.markers.end(), ma_cmp);
        float x_min, y_min, x_max, y_max;
        x_min=ma.markers.begin()->pose.position.x-ma.markers.begin()->scale.x*0.5;
        x_max=ma.markers.begin()->pose.position.x+ma.markers.begin()->scale.x*0.5;
        y_min=ma.markers.begin()->pose.position.y-ma.markers.begin()->scale.y*0.5;
        y_max=ma.markers.begin()->pose.position.y+ma.markers.begin()->scale.y*0.5;

        // if(id==3) ROS_INFO("1   n=%d  %.2f %.2f    %.2f %.2f", cloud.size(), x_max, y_min, y_max);

        cloud=Cloud_PassThrough(cloud, x_min, x_max, y_min, y_max, 0 ,0 );

        // if(id==3) ROS_INFO("2   n=%d  %.2f %.2f    %.2f %.2f", cloud.size(), x_max, y_min, y_max);
        if (cloud.points.size() > 0)
        {
            pcl::PointXYZI minpoint, maxpoint;
            pcl::getMinMax3D(cloud, minpoint, maxpoint);
            center_point[id].x = minpoint.x; 
        }

        sensor_msgs::PointCloud2 msgx;
        pcl::toROSMsg(cloud, msgx);
        msgx.header.frame_id = c_msg->header.frame_id;
        cloud_pub[id].publish(msgx);
        // printf("%d=%d\n", id, cloud.size());
    }    
}

//轮误差发布
void CLaserscan_Check::WheelErrPublish()
{
    std_msgs::Float32MultiArray err;
    //从ROS的参数服务器中获取laser_single_enable的值
    nh_local->getParam("laser_single_enable", laser_single_enable);

    float dx[2] = {999, 999};

    // 一边检测到轮胎，另一边相同值，过程流畅，有风险
    //得到中心点的坐标
    if(laser_single_enable)
    {
        if(center_point[2].x>0 && center_point[0].x<0)  center_point[0].x=center_point[2].x;
        if(center_point[0].x>0 && center_point[2].x<0)  center_point[2].x=center_point[0].x;
        if(center_point[1].x>0 && center_point[3].x<0)  center_point[3].x=center_point[1].x;
        if(center_point[3].x>0 && center_point[1].x<0)  center_point[1].x=center_point[3].x;
    }

    // ROS_INFO("%.2f %.2f", center_point[1].x, center_point[3].x);
    //计算误差
    if (center_point[0].x > 0 && center_point[2].x > 0)  dx[0] = -center_point[2].x + center_point[0].x;
    if (center_point[1].x > 0 && center_point[3].x > 0)  dx[1] =- center_point[1].x + center_point[3].x;

    static TDataFilter df_dx0(8), df_dx1(8);
    dx[0]=df_dx0.GetValue(dx[0]);
    dx[1]=df_dx1.GetValue(dx[1]);

    for(int i=0;i<2;i++)  err.data.push_back(dx[i]);
    //发布误差 
    wheel_err_pub.publish(err); 
    nodecheck->Find("node_rate")->Beat();
}
//雷达扫描1回调函数
void CLaserscan_Check::LaserScanCallback1(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(0, c_msg);
}
//雷达扫描2回调函数
void CLaserscan_Check::LaserScanCallback2(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(1, c_msg);
}
//雷达扫描3回调函数
void CLaserscan_Check::LaserScanCallback3(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(2, c_msg);
}
//雷达扫描4回调函数
void CLaserscan_Check::LaserScanCallback4(const sensor_msgs::LaserScan::ConstPtr &c_msg)
{
    CloudProcess(3, c_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserscan_check");   //节点
    ros::NodeHandle* node = new ros::NodeHandle("~");

    CLaserscan_Check lk(node);
    ros::Rate looprate(20);

    while (ros::ok())
    {
        lk.WheelErrPublish();
        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}
