#include <data_comm/paw_ctr.h>
#include <data_comm/paw_state.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#include <common/public.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <mqtt_comm/task.h>
#include <common/mydisplay.h>

using namespace std;

//取车夹爪控制状态机

class PAW_CTR
{
private:   //private 私有属性只能在基类中访问，不能在实例、派生类中访问
    string paw_ctr;
    double front_wheel_stop_distance=0.04, back_wheel_stop_distance=0.06;

    data_comm::paw_ctr paw_ctr_msg;
    data_comm::paw_state paw_state_msg;
    data_comm::car_state car_state_msg;

    TDataFilter center1_y, center3_y, center2_y, center4_y, center1_x, center3_x, center2_x, center4_x, len1, len2, len3, len4;
    mqtt_comm::task cur_task;
    ////创建 ros 节点句柄
    ros::NodeHandle *nh;
    //订阅
    ros::Subscriber scanlidar_pos_sub, paw_state_sub, car_state_sub, task_sub, remainpath_sub, movedir_sub, agv_allin_sub, obs_dis_sub;
    float remain_path_length=0;

    ros::Publisher paw_ctr_pub,work_speedlimit_pub;
    bool agv_allin_flag=false, agv_in_flag=false;
    float maxspeed_agvin=0.8;
    float car_distance=999;
    int lane_type_init=0;
    bool baojia_enable=true;
    string move_dir="";

    TNodeCheck *nodecheck;

public:  //public：可以继承、实例化

    //函数声明
    void ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void PawStateCallback(const data_comm::paw_state::ConstPtr &msg);
    void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void MoveDirCallback(const std_msgs::String::ConstPtr &msg);
    void AgvInCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void ObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

    void Run();

    PAW_CTR(): center1_x(2), center3_x(2), center2_x(2), center4_x(2), center1_y(2), center3_y(2), center2_y(2), center4_y(2), len1(2), len2(2), len3(2), len4(2)
    {
        nh = new ros::NodeHandle("~");

        nodecheck=new TNodeCheck(nh, "node_rate");
	    nodecheck->Find("node_rate")->SetLimit(15);

        // paw_state_str = "baojia_done";
        // paw_state_str = "wait_for_paw_drop";
        paw_ctr = "paw_idle";
        nh->getParam("front_wheel_stop_distance", front_wheel_stop_distance);
        nh->getParam("back_wheel_stop_distance", back_wheel_stop_distance);
        nh->getParam("baojia_enable", baojia_enable);
        nh->getParam("maxspeed_agvin", maxspeed_agvin);

        scanlidar_pos_sub = nh->subscribe<std_msgs::Float32MultiArray>("/laserscan_check/wheel_distance", 10, &PAW_CTR::ScanPosCallback, this);
        paw_state_sub = nh->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, &PAW_CTR::PawStateCallback, this);
        car_state_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &PAW_CTR::CarStateCallback, this);
        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &PAW_CTR::TaskCallback, this);
        remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &PAW_CTR::RemainPathCallback, this);
        movedir_sub = nh->subscribe<std_msgs::String>("/move_dir", 10, &PAW_CTR::MoveDirCallback, this);
        agv_allin_sub = nh->subscribe<std_msgs::Int32MultiArray>("/cloud_calculation_inside/agv_in_flag", 10, &PAW_CTR::AgvInCallback, this);
        // agv_allin_sub = nh->subscribe<std_msgs::Int32MultiArray>("/cloud_calculation/agv_in_flag", 10, &PAW_CTR::AgvInCallback, this);
        
        // obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation/obs_dis", 10, &PAW_CTR::ObsDisCallback, this);
        obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/pathtrack/car_in_out_dis", 10, &PAW_CTR::ObsDisCallback, this);

        paw_ctr_pub = nh->advertise<data_comm::paw_ctr>("/pawcontrol/paw_ctr", 10);
        work_speedlimit_pub = nh->advertise<std_msgs::Float64>("/pawcontrol/work_speedlimit", 10);


        paw_ctr_msg.left_paw_distance_control = 0;
        paw_ctr_msg.left_paw_speed = 0;
        paw_ctr_msg.right_paw_distance_control = 0;
        paw_ctr_msg.right_paw_speed = 0;
        paw_ctr_msg.paw_lift_control = 0;
        paw_ctr_msg.vehicle_put_control = 0;

        paw_state_msg.updown_state = 0;
        paw_state_msg.carhold_state = 0;
        paw_state_msg.left_axischange_state = 0;
        paw_state_msg.right_axischange_state = 0;
    }
};

//扫描4个中心点位置
void PAW_CTR::ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    center1_x.GetValue(msg->data[0]),  center1_y.GetValue(msg->data[1]),  len1.GetValue(msg->data[2]);
    center2_x.GetValue(msg->data[3]),  center2_y.GetValue(msg->data[4]),  len2.GetValue(msg->data[5]);
    center3_x.GetValue(msg->data[6]),  center3_y.GetValue(msg->data[7]),  len3.GetValue(msg->data[8]);
    center4_x.GetValue(msg->data[9]),  center4_y.GetValue(msg->data[10]), len4.GetValue(msg->data[11]);

    // printf("%.2f %.2f %.2f %.2f\n", len1.value, len2.value, len3.value, len4.value);
}
//爪子状态回调函数
void PAW_CTR::PawStateCallback(const data_comm::paw_state::ConstPtr &msg)
{
    paw_state_msg = *msg;
}
//转运车状态回调函数
void PAW_CTR::CarStateCallback(const data_comm::car_state::ConstPtr &msg)
{
    car_state_msg = *msg;
}
//障碍物距离回调函数
void PAW_CTR::ObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    car_distance=msg->data[0];
}

//  夹爪接收任务指令
void PAW_CTR::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    cur_task = *msg;   //获取当前任务
    //"paw_idle" 夹爪无动作，baojia_done 夹爪上升？ wait_for_paw_drop 夹爪放下
    bool new_task_enable = (paw_ctr=="paw_idle" || paw_ctr=="baojia_done" || paw_ctr=="wait_for_paw_drop");
    //夹爪是否动作
    new_task_enable = new_task_enable && msg->path.size()>1; 

    //抬起
    if (msg->cmd == "pick task")
    {
        //等待爪子放下
        if(new_task_enable)  paw_ctr = "wait_for_paw_drop"; //"wait_for_paw_drop";  //"paw_drop_done";
    }
    //放下
    else if (msg->cmd == "release task")
    {   
        if(new_task_enable)  paw_ctr = "baojia_done";
    } 
    else
    {
        paw_ctr="paw_idle";   //夹爪无动作
    }   
}

//夹爪保持路径的回调函数
void PAW_CTR::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length=msg->data;
}
//夹爪移动方向的回调函数

void PAW_CTR::MoveDirCallback(const std_msgs::String::ConstPtr &msg) // 接收控制信号
{
    move_dir=msg->data;
}
//  ???
void PAW_CTR::AgvInCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (msg->data.size() == 2)
    {        
        agv_in_flag = msg->data[0];
        agv_allin_flag = msg->data[1]; // agv_all_in
        // printf("flag=%d\n",agv_allin_flag);
    }
}

/*
传感器配置
3------2
|      |--->车头朝向
1------4
*/

//// 取车夹爪控制状态机
void PAW_CTR::Run()   ////  主运行函数
{
    nodecheck->Find("node_rate")->Beat();
    // if(cur_task.subcmd=="3") return;  //缓停

    float time_spend = ros::Time::now().toSec() - cur_task.stamp.toSec();   //把cur_task时间戳转化成浮点型格式
    float drop_length = 10;
    float speedlimit=999;
    float mistop_speedlimit = 999;

    nh->getParam("/mi_stop/speedlimit", mistop_speedlimit);
    nh->getParam("/local_path_plan/lane_type", lane_type_init);

    // printf("remain_path=%.2f\n",remain_path_length);
    // printf("time_spend=%.2f\n",time_spend);

    // 取车夹爪控制状态机

    bool robot_in_field = (lane_type_init==23 || lane_type_init==24 || lane_type_init==25 || lane_type_init==31);
   
    //抬起任务 && 车状态不是初始化状态 && cur_task.final_path
    if (cur_task.cmd == "pick task" && car_state_msg.ctrmode && cur_task.final_path)
    {
        if(cur_task.subcmd=="2") drop_length=7;  //立体车库有坡度，距离短点
        else drop_length=12;

        //
        if (paw_ctr == "wait_for_paw_drop" && time_spend > 0.8 && (remain_path_length < drop_length || (car_distance<drop_length*0.5 && robot_in_field)) && remain_path_length >1)
        {
            paw_ctr_msg.paw_lift_control = 2;  //夹爪降低
            paw_ctr_msg.vehicle_put_control = 0; // 0x00无动作
            paw_ctr = "paw_dropping";   //夹爪正在放下
        }
        //夹爪正在放下         paw_state_msg=2 放下
        else if (paw_ctr == "paw_dropping" && paw_state_msg.updown_state == 2)
        {
            paw_ctr_msg.paw_lift_control = 0;  //夹爪无动作
            paw_ctr = "paw_drop_done";   夹爪已放下
        }
        else if (paw_ctr == "paw_drop_done")    //夹爪已放下
        {
            if(agv_allin_flag)   speedlimit=0.07;  //  车辆完全进入后要立刻降低速度
            else  speedlimit=maxspeed_agvin;   //  车辆开始对接后要降低速度
               
            float len=0.5*(len2.value+len4.value);
            // printf("len=%.2f\n",len);
            if(len>0.1 && agv_allin_flag)     
            {
                float center_y=0.5*(center2_y.value+center4_y.value);
                
                if (center_y>-front_wheel_stop_distance && move_dir=="front")  paw_ctr = "car_stop";
                else if(center_y<front_wheel_stop_distance && move_dir=="back")  paw_ctr = "car_stop";

                // ROS_INFO("dir=%s y=%.2f sop_dis=%.2f", move_dir.c_str(), center_y, front_wheel_stop_distance);
            }
        }
        //搬运车停止，且速度小于0.01
        else if (paw_ctr == "car_stop" && fabs(car_state_msg.speed[0]) < 0.01)
        {
            paw_ctr = "length_change_start";   //改变爪子长度
            // ROS_INFO("STOP:%.3f %.3f", center2_y.value, center4_y.value);
        }
        //改变夹爪长度
        else if (paw_ctr == "length_change_start") // TODO 分段
        {
            paw_ctr_msg.left_paw_speed = 0;
            float center_y = 0.5 * (center1_y.value + center3_y.value);
            float len=0.5*(len1.value+len3.value);
            if(fabs(center_y)>0.1)  paw_ctr_msg.left_paw_speed = 3; // 高速
            else paw_ctr_msg.left_paw_speed = 1; // 低速

            // 方向设置
            if (center_y > back_wheel_stop_distance)  paw_ctr_msg.left_paw_distance_control = 1; // 增加
            else if (center_y < -back_wheel_stop_distance)  paw_ctr_msg.left_paw_distance_control = 2; // 缩短
            else if (len>0.1) 
            {
                paw_ctr_msg.left_paw_distance_control = 0,  paw_ctr="length_change_done";   //"baojia_done" 不取车
                nh->setParam("/write_txt/test_saveflag", true);
            }
            paw_ctr_msg.right_paw_distance_control = 0; // TODO
            paw_ctr_msg.right_paw_speed = 0;
        }
        //夹爪长度改变结束，且baojia_enable=1
        else if (paw_ctr == "length_change_done" && baojia_enable)
        {
            if (paw_state_msg.left_axischange_state==0 || paw_state_msg.left_axischange_state==2 || paw_state_msg.left_axischange_state==4)
            {
                paw_ctr_msg.vehicle_put_control = 0xAA; // 0xAA自动取车
                paw_ctr_msg.right_paw_distance_control = 0; 
                paw_ctr = "baojia_start";
            }
        }
        //夹爪保驾开始 ， carhold_stat=2
        else if (paw_ctr == "baojia_start" && paw_state_msg.carhold_state == 2)
        {
            paw_ctr = "baojia_done";   //包夹结束
            paw_ctr_msg.vehicle_put_control = 0; // 0x00无动作
        }
    }
    //放车夹爪控制状态机
    //  //抬起任务 && 车状态不是初始化状态 && cur_task.final_path
    else if (cur_task.cmd == "release task" && car_state_msg.ctrmode && cur_task.final_path) // 放车夹爪控制状态机
    {
        static bool first_save=false;
        if (paw_ctr == "baojia_done" && time_spend > 0.8 && (remain_path_length < 0.05 || mistop_speedlimit < 0.01) && fabs(car_state_msg.speed[0]) < 0.01)
        {
            if(!first_save)
            {
                nh->setParam("/write_txt/test_saveflag", true);
                first_save=true;
            }
            paw_ctr_msg.vehicle_put_control = 0xBB;     // 0xBB自动放车
            paw_ctr = "car_dropping";
        }
        else if (paw_ctr == "car_dropping" && paw_state_msg.carhold_state == 4)
        {
            first_save=false;
            paw_ctr = "wait_for_paw_drop";         //"car_drop_done";
            paw_ctr_msg.vehicle_put_control = 0;   // 0x00无动作
        }
    }
    //空路径
    else if((cur_task.cmd == "release task" || cur_task.cmd == "pick task") && cur_task.path.size()<2 &&!cur_task.final_path)  //空路径
    {
        // ROS_INFO("path blank");
    }
    //搬运车和夹爪无动作状态
    else if(car_state_msg.ctrmode && cur_task.cmd!="pick task" && cur_task.cmd!="release task")   //  其它指令任务应清除夹爪作业动作     if(cur_task.cmd=="") 
    {
        paw_ctr_msg.vehicle_put_control = 0; // 0x00无动作
        paw_ctr_msg.paw_lift_control = 0; 
        paw_ctr = "paw_idle";
    }

    // printf("paw=%s\n", paw_state_str.c_str());
    
    static TTimer tmr;
    if(tmr.GetValue()>0.1)  
    {
        nh->setParam("paw_state", paw_ctr);
        tmr.Clear();
    }
    //发布夹爪控制状态
    paw_ctr_pub.publish(paw_ctr_msg);

    std_msgs::Float64 work_speedlimit_msgs;
    work_speedlimit_msgs.data=speedlimit;
    work_speedlimit_pub.publish(work_speedlimit_msgs);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pawcontrol");   //初始化节点

    PAW_CTR pawctr;
    ros::Rate looprate(30);   //自循环频率
    while (ros::ok())
    {
        pawctr.Run();

        //当程序运行到spinOnce()时，程序到相应的topic订阅缓存区查看是否存在消息，
        //如果有消息，则将消息传入回调函数执行回调函数;如果没有消息则继续向后执行。
        ros::spinOnce(); 
                       
        looprate.sleep();
    }
    return 0;
};