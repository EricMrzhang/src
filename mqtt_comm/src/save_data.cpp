#include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_ros/transforms.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/common.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include "common/public.h"
#include <common/mydisplay.h>
#include <mqtt_comm/task.h>
#include <data_comm/car_state.h>
#include <data_comm/car_angle.h>
#include <pathtrack/track_state.h>
#include <fstream>
#include <data_comm/car_ctr.h>
#include <gps/MyGPS_msg.h>
#include <string>
#include <algorithm> // for std::remove

using namespace std;

TNodeCheck *nodecheck;
geometry_msgs::PointStamped picar_pose;
int n=0;
std_msgs::Float64 cardis;
string paw_state;
data_comm::car_state car_state;
data_comm::car_angle car_angle;
data_comm::car_ctr car_ctr;
double record_time = 0;


struct Data_Record
{
    geometry_msgs::Point target_pos,  actual_pos;
    geometry_msgs::Point scan_data[4];
    geometry_msgs::PointStamped dst_p;
    mqtt_comm::task task;
};
Data_Record dr;

void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg);
void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
void PassedPathCallback(const std_msgs::Float64::ConstPtr &msg);
void WheelErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
void BodyErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
// void PawStateCallback(const data_comm::paw_state::ConstPtr &msg);
void ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
void TrackStateCallback(const pathtrack::track_state::ConstPtr &msg);
void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
void CarAngleCallback(const data_comm::car_state::ConstPtr &msg);
void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg);
void GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg);
void aimpointCallback(const geometry_msgs::PointStamped::ConstPtr &msg);



string filename;

void initfile(string id)
{
    ofstream fs(filename);

    fs<<id;
    fs<<"time (task task_subcmd) (target.x target.y target.angl) (actual.x actual.y actual.angl) (car_ctr.speed car_ctr.angle) car_state.speed wheel_angle (2_x 2_y 4_x 4_y) (point.x point.y)\n";
    fs.close();
}

std::string removeChar(std::string str, char c) {
    // 使用 std::remove 将所有 c 字符移动到字符串的末尾
    std::remove(str.begin(), str.end(), c);

    // 使用 erase 移除所有移动后的字符
    str.erase(std::remove(str.begin(), str.end(), c), str.end());

    return str;
}


void savedata(bool err_flag)
{
    // geometry_msgs::PointStamped p_base, p_map;
    // p_base.header.frame_id="base_link";
    // p_base.header.stamp=ros::Time::now();
    // transformPoint("map", p_base, p_map, "XXX");
    // dr.actual_pos=p_map.point;
    float mi_dis=P2P(dr.actual_pos.x,dr.actual_pos.y,picar_pose.point.x,picar_pose.point.y);
        
    char buf[1000];
    std::string num = dr.task.subcmd.c_str();//检测变量是否含有1

    // 移除所有的 '1'
    num = removeChar(num, '1');
    
    //nh->getParam("err_record_flag", flag_err);
    if(err_flag==true){
       sprintf(buf,"%s %s %s %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.3f %.3f %.3f %.3f %.2f %.2f\n",
            NowtimetoString().c_str(),
            dr.task.cmd.c_str(),num.c_str(),//dr.task.subcmd.c_str(),
            dr.target_pos.x, dr.target_pos.y, dr.target_pos.z, 
            dr.actual_pos.x, dr.actual_pos.y, dr.actual_pos.z,
            car_ctr.speed,car_ctr.angle,
            car_state.speed[0],car_angle.angle[4],dr.scan_data[1].x, dr.scan_data[1].y,dr.scan_data[3].x, dr.scan_data[3].y,
            dr.dst_p.point.x,dr.dst_p.point.y);
    }
    if(err_flag==false){
        sprintf(buf,"%s %s %s %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %d %d %d %d %.2f %.2f\n",
            NowtimetoString().c_str(),
            dr.task.cmd.c_str(),num.c_str(),//dr.task.subcmd.c_str(),
            dr.target_pos.x, dr.target_pos.y, dr.target_pos.z, 
            dr.actual_pos.x, dr.actual_pos.y, dr.actual_pos.z,
            car_ctr.speed,car_ctr.angle,
            car_state.speed[0],car_angle.angle[4],0,0,0,0,
            dr.dst_p.point.x,dr.dst_p.point.y);
    }

    // sprintf(buf,"%s %s %s %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
    //         NowtimetoString().c_str(),
    //         dr.task.cmd.c_str(),num.c_str(),//dr.task.subcmd.c_str(),
    //         dr.target_pos.x, dr.target_pos.y, dr.target_pos.z, 
    //         dr.actual_pos.x, dr.actual_pos.y, dr.actual_pos.z,
            
    //         // dr.scan_data[0].x, dr.scan_data[0].y, dr.scan_data[1].x, dr.scan_data[1].y, 
    //         // dr.scan_data[2].x, dr.scan_data[2].y, dr.scan_data[3].x, dr.scan_data[3].y,
    //         // mi_dis,
    //         car_ctr.speed,car_ctr.angle,
    //         car_state.speed[0],car_angle.angle[4],dr.scan_data[1].x, dr.scan_data[1].y,dr.scan_data[3].x, dr.scan_data[3].y,
    //         dr.dst_p.point.x,dr.dst_p.point.y);
    // // fs<<buf;

    ofstream fs(filename,ios::app);
    fs<<buf;
    fs.close();
}

void TrackStateCallback(const pathtrack::track_state::ConstPtr &msg)
{
    // printf("111111111\n");
}

void TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    dr.task = *msg;
    
    if(msg->path.size())
    {
        dr.target_pos.x=msg->path.back().pointX;
        dr.target_pos.y=msg->path.back().pointY;
        dr.target_pos.z=msg->path.back().pointHA;
    }    
}

void ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    dr.scan_data[0].x=msg->data[0],  dr.scan_data[0].y=msg->data[1];
    dr.scan_data[1].x=msg->data[3],  dr.scan_data[1].y=msg->data[4];
    dr.scan_data[2].x=msg->data[6],  dr.scan_data[2].y=msg->data[7];
    dr.scan_data[3].x=msg->data[9],  dr.scan_data[3].y=msg->data[10];
}

void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg)
{
    car_ctr=*msg;
}

void CarStateCallback(const data_comm::car_state::ConstPtr &msg)
{
    car_state=*msg;
}

void CarAngleCallback(const data_comm::car_angle::ConstPtr &msg)
{
    car_angle=*msg;
}

void CardisCallback(const std_msgs::Float64::ConstPtr &msg)
{
    cardis=*msg;
}

void GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg)
{

	dr.actual_pos.x = msg->map_x;
	dr.actual_pos.y = msg->map_y;
	dr.actual_pos.z = msg->Angle;

}

void aimpointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    dr.dst_p = *msg;
}
// void set_saveflag()
// {
//     if(dr.task.cmd=="pick task")
//     {
//         if(paw_state=="length_change_start")
//     }
//     else if(dr.task.cmd=="release task")
//     ;
//     else if(dr.task.cmd=="move task")


// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_data");
    ros::NodeHandle nh("~");

    // nodecheck = new TNodeCheck(&nh, "node_rate");

    // ros::Subscriber car_distance_sub = nh.subscribe<std_msgs::Float32MultiArray>("/cloud_calculation/obs_dis", 10, CarDistanceCallback);
    ros::Subscriber task_sub = nh.subscribe<mqtt_comm::task>("/task_cmd", 10, TaskCallback);
    ros::Subscriber scan_pos_sub = nh.subscribe<std_msgs::Float32MultiArray>("/laserscan_check/wheel_distance", 10, ScanPosCallback);
    
    ros::Subscriber track_state_sub = nh.subscribe<pathtrack::track_state>("/pathtrack/track_state", 10, TrackStateCallback);
    ros::Subscriber carctr_sub = nh.subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, CarCtrCallback);
    ros::Subscriber carstate_sub = nh.subscribe<data_comm::car_state>("/can_comm/car_state", 10, CarStateCallback);
    ros::Subscriber	carangle_sub = nh.subscribe<data_comm::car_angle>("/can_comm/car_angle", 10, &CarAngleCallback); //创建车轮角度订阅者

    ros::Subscriber car_distance_sub = nh.subscribe<std_msgs::Float64>("/mi_stop/car_dis",10,CardisCallback);

    ros::Subscriber	gps_sub = nh.subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, &GPSDataCallback);
    ros::Subscriber aimpoint = nh.subscribe<geometry_msgs::PointStamped>("/pathtrack/aimpoint", 10, &aimpointCallback);

    filename="/home/bit/byc_data/"+NowtimetoString()+".txt";  //to_string(int(ros::Time::now().toSec()))+".txt";
    

    picar_pose.point.x=-91.50;
    picar_pose.point.y=-34.73;
    picar_pose.point.z=-87.7;


    nh.setParam("save_flag", true);
    nh.setParam("/save_date_flag", false);

    //nh.setParam("/pathtrack/err_record_flag", false);  //初始化
    bool flag_err=false;

    ros::Rate looprate(20);
    
    while (ros::ok())
    {
        // nodecheck->Find("node_rate")->Beat();
        nh.getParam("/pawcontrol/paw_state", paw_state);

        if(paw_state=="length_change_start" ) flag_err=true;
        if(paw_state=="length_change_done" ) flag_err=false;

        bool flag1=true;
        bool flag2=false;
        nh.getParam("save_flag", flag1);
        nh.getParam("/save_date_flag", flag2);

        
        //nh.getParam("/pathtrack/err_record_flag", flag_err); 

        
        string agvId;
        nh.getParam("/agvId", agvId);

        if(flag2)  
        {
            if(n==0)
            {
                initfile(agvId);
                n=1;
            }
            nh.setParam("save_flag", true);
            
            static TTimer tr;
	        double start_record_time;
	        record_time=tr.GetValue();

	        if(record_time>1){
                savedata(flag_err);
                tr.Clear();
	        };
            
        }
        
        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}