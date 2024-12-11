#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "common/public.h"
#include <common/mydisplay.h>
#include <mqtt_comm/task.h>

#include <vector>
#include <algorithm>

using namespace std;

//停车过程
//stop_ctr_state:1减速计时，2减速，3速度减下来了，4按距离停车
geometry_msgs::Point start_pos;
float stop_distance=0.3, start_distance=999;
mqtt_comm::task cur_task;

int stop_ctr_state=-1;
float car_distance;

TNodeCheck *nodecheck;

//将一个在base_link坐标系下的点转换到map坐标系下
void GetPosInMap(geometry_msgs::Point &p)
{
    geometry_msgs::PointStamped base_p, map_p;
    base_p.header.frame_id="base_link";
    base_p.header.stamp=ros::Time::now();
    transformPoint("map",base_p, map_p);
    p=map_p.point;
}
//距离回调函数
void CarDistanceCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(cur_task.cmd=="release task" && cur_task.subcmd=="1" && cur_task.final_path) 
    {
        car_distance=msg->data[0];  //车辆到物体的距离
        // printf("car_dis=%.2f\n",car_distance);
        if(car_distance>3)  stop_ctr_state=0; 
        else if(stop_ctr_state==0)  stop_ctr_state++;
    }
}

//计算运行路程
float GetRunLength()
{
    float res=-1;
    
    geometry_msgs::Point cur_pos;
    GetPosInMap(cur_pos);
    res=GetDistanceXY(start_pos, cur_pos);
    
    return res;
}

//  接收任务指令  回调函数
void TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    if(msg->cmd.find("task")!=string::npos)
    {                
        cur_task = *msg;
        stop_ctr_state=0;   //不停车
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mi_stop");
    ros::NodeHandle nh("~");

    nodecheck = new TNodeCheck(&nh, "node_rate");

    sleep(1);

    // ros::Subscriber car_distance_sub = nh.subscribe<std_msgs::Float32MultiArray>("/cloud_calculation/obs_dis", 10, CarDistanceCallback);
    //订阅话题
    ros::Subscriber car_distance_sub = nh.subscribe<std_msgs::Float32MultiArray>("/pathtrack/car_in_out_dis", 10, &CarDistanceCallback);
    ros::Subscriber task_sub = nh.subscribe<mqtt_comm::task>("/task_cmd", 10, TaskCallback);
    //发布话题
    ros::Publisher dis_pub = nh.advertise<std_msgs::Float64>("car_dis",10);
    ros::Publisher stop_state_pub = nh.advertise<std_msgs::Int32>("stop_ctr_state",10);

    TTimer tmr;
    ros::Rate looprate(20);
    while (ros::ok())
    {
        nodecheck->Find("node_rate")->Beat();

        float speedlimit=999;
        if(stop_ctr_state==1)  //  开始计时
        {

            tmr.Clear();
            stop_ctr_state++;
        }
        else if(stop_ctr_state==2)  //  减速
        {
            speedlimit=0.05;
            if(tmr.GetValue()>2)  //  一段时间后,认为速度降下来了
            {
                start_distance=car_distance;
                GetPosInMap(start_pos);
                stop_ctr_state++;
                // printf("start_dis=%.2f\n",start_distance);
            }
        }
        else if(stop_ctr_state==3)   //  按距离停车
        {
            float run_length=GetRunLength();
            float dis= start_distance-run_length-stop_distance;

            std_msgs::Float64 car_dis;
            car_dis.data=dis+stop_distance;
            dis_pub.publish(car_dis);
            
            speedlimit=0.1;
            if(dis>0.5)  speedlimit=0.3;
            else if(dis>0.2)  speedlimit=0.1;
            else if(dis>0.1)  speedlimit=0.06;
            else speedlimit=0,  stop_ctr_state++;
            // ROS_INFO("speedlimit=%.2f,dis=%.3f,runlength=%.3f,start_distance=%.3f", speedlimit,dis,run_length,start_distance);
        }
        else if(stop_ctr_state==4)
        {
            speedlimit=0.0;
        }
        // printf("spd=%.2f ctr=%d\n",speedlimit,stop_ctr_state);
        nh.setParam("speedlimit", speedlimit);

        std_msgs::Int32 msg;
        msg.data=stop_ctr_state;
        stop_state_pub.publish(msg);

        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}