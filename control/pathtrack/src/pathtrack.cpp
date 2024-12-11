#include <ros/ros.h>
#include <fstream>   //文件流
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Path.h>   //常用于机器人导航
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <common/public.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <data_comm/car_ctr.h>
#include <data_comm/car_state.h>
#include <data_comm/paw_state.h>
\
#include <mqtt_comm/task.h>
#include <mqtt_comm/iot_taskpath.h>

#include <stdlib.h>
#include <pathtrack/track_state.h>
// #include "mqtt_comm/resp_agvstate.h"
#include "mqtt_comm/resp_iot.h"
#include "gps/MyGPS_msg.h"
#include <common/mydisplay.h>

using namespace std;

// #define selfturn_angle_err 40

//跟踪状态 结构体
struct track_state
{
    string stopReason;
    int timestamp;

    int self_turn_ctr;
    int turnmode_ctr;
    float track_angle_err;
    float track_dis_err;

    float aim_range;
    float steering_property;

    mqtt_comm::task cur_task;
    data_comm::car_ctr car_ctr;
    data_comm::car_state car_state;
};

class TPathTrack
{
private:
    ros::NodeHandle *nh_local, *nh;
    //发布话题
    ros::Publisher aimpoint_pub, carctr_pub, sp_distance,track_err_pub,track_state_pub,move_dir_pub,car_dis_pub,obs_speedlimit_pub, stop_reason_pub, selfturn_ctr_pub; 
    //订阅话题
    ros::Subscriber localpath_sub, carstate_sub, remainpath_sub, passedpath_sub, task_sub, wheel_err_sub, paw_state_sub, obs_speedlimit_sub, obs_dis_sub, agv_in_sub;
    ros::Subscriber resp_iot_sub;
    ros::Subscriber body_front_err_sub,body_back_err_sub ;
    ros::Subscriber obs_front_speedlimit_sub,obs_back_speedlimit_sub,obs_left_speedlimit_sub,obs_right_speedlimit_sub, work_speedlimit_sub;
    ros::Subscriber front_obs_dis_sub,back_obs_dis_sub,inside_obs_dis_sub,gps_sub;
    
    //创建nav_msgs::Path对象
    nav_msgs::Path localpath;
    
    int self_turn_ctr = 0, turnmode_ctr = 0;
    data_comm::car_state cur_carstate;
    
    mqtt_comm::task cur_task;
    mqtt_comm::iot_taskpath task_path;

    TNodeCheck *nodecheck;
    
    float angle_offset = 0;
    bool test_flag = false;
    float test_speed = 0;

    geometry_msgs::PointStamped aimpoint;
    float aim_range = 2, ref_speed = 0.6; 
    float steering_property = 1;
    float track_angle_err = 0, orien_angle_err = 0;
    bool run_enable = false;
    bool pub_enable = false;
    bool iterative_ctr_enable =false;
    string paw_state;
    float remain_path_length = 0;
    float passed_path_length = 0;
    float turn_speed_max = 0;

    bool obs_stop_enable=true, laser_work_enable = true, agv_in_flag=false;
    float speedlimit = 999, obstacle_speedlimit = 999;
    float work_speedlimit = 999, mistop_speedlimit = 999;
    float front_middle_speedlimit = 999, back_middle_speedlimit = 999, front_side_speedlimit=999, back_side_speedlimit=999;
    float left_speedlimit = 999, right_speedlimit = 999;
    float inside_err = 999, front_err = 999, back_err = 999, inside_left_dis = 999, inside_right_dis = 999;
    float body_check_err=999, wheel_check_err=999;
    float front_car_distance=999,back_car_distance=999,car_distance=999;
    float inside_front_dis,inside_back_dis;

    mqtt_comm::resp_iot agv_state;
    data_comm::paw_state paw_state_msg;
    data_comm::car_ctr car_ctr;
    string move_dir="back", stop_str="";
    geometry_msgs::Point GPS_msg;

    pathtrack::track_state cur_ts;
    int n_save=0;
    int iterative_ctr_flag=0;
    int lane_type_init=0;
    bool selfturn_disable=0;
    float DeviationControl_Kp=15;

public:
    TPathTrack()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();
        //点云发布
        aimpoint_pub = nh_local->advertise<geometry_msgs::PointStamped>("aimpoint", 10); // 发布预瞄点
        
        //节点检查
        nodecheck = new TNodeCheck(nh_local, "node_rate track_err_rate");
        nodecheck->Find("node_rate")->SetLimit(10);
        //发布控制指令
        carctr_pub = nh_local->advertise<data_comm::car_ctr>("ctr_cmd", 10); // 发布控制信息
        //发布跟踪误差
        track_err_pub = nh_local->advertise<std_msgs::Float64>("track_err",10);
       //发布跟踪状态
        track_state_pub = nh_local->advertise<pathtrack::track_state>("track_state",10);
       //
        car_dis_pub = nh_local->advertise<std_msgs::Float32MultiArray>("car_in_out_dis",10);
      //发布运动方向
        move_dir_pub = nh_local->advertise<std_msgs::String>("/move_dir",10);
        
        stop_reason_pub = nh_local->advertise<std_msgs::String>("/stop_reason",10);
       
        selfturn_ctr_pub = nh_local->advertise<std_msgs::Int32>("/selfturn_ctr",10);

        obs_speedlimit_pub = nh_local->advertise<std_msgs::Float64>("/obs_speedlimit",10);

        gps_sub = nh->subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, &TPathTrack::GPSDataCallback,this);
        localpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, &TPathTrack::LocalPathCallback, this);
        carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &TPathTrack::CarStateCallback, this);
        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TPathTrack::TaskCallback, this);
       //局部路径规划
        remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &TPathTrack::RemainPathCallback, this);
        passedpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/passedpath", 10, &TPathTrack::PassedPathCallback, this);
        //点云计算
        body_front_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/agv_lidar_err", 10, &TPathTrack::BodyFrontErrCallback, this);
        body_back_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_back/agv_lidar_err", 10, &TPathTrack::BodyBackErrCallback, this);
       //车轮误差
        wheel_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/laserscan_check_angle/wheel_err", 10, &TPathTrack::WheelErrCallback, this);
        
        // obs_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation/obs_speedlimit", 10, &TPathTrack::ObsSpeedlimitCallback, this);
        
        //计算车体周围的障碍物？
        obs_front_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/obs_speedlimit", 10, &TPathTrack::Front_ObsSpeedlimitCallback, this);
        obs_back_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_back/obs_speedlimit", 10, &TPathTrack::Back_ObsSpeedlimitCallback, this);
        obs_left_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_left/obs_speedlimit", 10, &TPathTrack::Left_ObsSpeedlimitCallback, this);
        obs_right_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_right/obs_speedlimit", 10, &TPathTrack::Right_ObsSpeedlimitCallback, this);
        //夹爪速度限制
        work_speedlimit_sub = nh->subscribe<std_msgs::Float64>("/pawcontrol/work_speedlimit", 10, &TPathTrack::WorkSpeedlimitCallback, this);
        // obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation/obs_dis", 10, &TPathTrack::ObsDisCallback, this);
       ////计算车体周围的障碍物的距离
        front_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/obs_dis", 10, &TPathTrack::FrontObsDisCallback, this);
        back_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_back/obs_dis", 10, &TPathTrack::BackObsDisCallback, this);
        inside_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_inside/car_dis", 10, &TPathTrack::InsideObsDisCallback, this);
        
        //夹爪状态
        paw_state_sub = nh->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, &TPathTrack::PawStateCallback, this);
        // agv_in_sub = nh->subscribe<std_msgs::Int32MultiArray>("/cloud_calculation/agv_in_flag", 10, &TPathTrack::AgvInCallback, this);
        //agv
        agv_in_sub = nh->subscribe<std_msgs::Int32MultiArray>("/cloud_calculation_inside/agv_in_flag", 10, &TPathTrack::AgvInCallback, this);

        resp_iot_sub = nh->subscribe<mqtt_comm::resp_iot>("/resp_iot", 10, &TPathTrack::RespIOTCallback, this);
        aimpoint.header.frame_id = "";
        //角度补偿
        nh_local->getParam("angle_offset", angle_offset);
        //测试速度
        nh_local->getParam("test_speed", test_speed);
        //纠偏控制kp
        nh_local->getParam("deviationcontrol_kp", DeviationControl_Kp);

        test_flag=(fabs(test_speed)>0.01);
    };

    //函数声明
    void GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg);
    void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void PassedPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void WheelErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void BodyFrontErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void BodyBackErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

    void Front_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Back_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Left_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void Right_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void WorkSpeedlimitCallback(const std_msgs::Float64::ConstPtr &msg);

    // void ObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void FrontObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void BackObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void InsideObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

    void AgvInCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void PawStateCallback(const data_comm::paw_state::ConstPtr &msg);
    void RespIOTCallback(const mqtt_comm::resp_iot::ConstPtr &msg);

    void Run();
    int SelfTurnCtr();
    int MoveModeCtr();
    void UpdateCtrParam(float vel);
    void GetAimAngleErr(bool is_hengyi_);
    int StopCtr();
    int StopCheck();
    int CheckMoveMode();
    void CheckSpeedLimit();
    void CheckObsSpeedLimit();
    void PubCarCtr(data_comm::car_ctr ctr);
    float SpeedLimitByCurve(float len);
    
    void PurePursuit(data_comm::car_ctr &car_ctr);
    void DeviationControl(data_comm::car_ctr &car_ctr);
    void TS_Publish();
    int FaultCheck();
    void FindNearestPointInPath(mqtt_comm::iot_taskpath taskpath, int &start_id, float max_len);
    //检查跟踪误差
    float CheckTrackErr()
    {
        float res=0;
        if(localpath.poses.size()>0)  res=GetDistance(*localpath.poses.begin());
        return res;
    }
};
//IOT 物联网
void TPathTrack::RespIOTCallback(const mqtt_comm::resp_iot::ConstPtr &msg)
{
    agv_state = *msg;
}
//GPS数据
void TPathTrack::GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg)
{
    GPS_msg.x=msg->map_x;
    GPS_msg.y=msg->map_y;
    GPS_msg.z=msg->Angle;
}

//夹爪状态
void TPathTrack::PawStateCallback(const data_comm::paw_state::ConstPtr &msg)
{
    paw_state_msg = *msg;
}
//弯道限速   ，根据角度d_angle大小，取不同的res值
float TPathTrack::SpeedLimitByCurve(float len)
{
    float res=999;
    if(localpath.poses.size()<5) return res;

    float a0=GetYawFromPose(localpath.poses.front())*180/M_PI;
    float d_angle=0;
    float s=0;
    for(int i=0;i<localpath.poses.size()-3;i++)
    {
        float ds=GetDistanceXY(localpath.poses[i].pose.position, localpath.poses[i+1].pose.position);
        s+=ds;

        float a1=GetYawFromPose(localpath.poses[i])*180/M_PI;
        d_angle=max(d_angle, fabs(a1-a0));
        if(s>len)  break;
    }
        
    if(d_angle>50)  res=0.8;
    else if(d_angle>30) res=1.0;
    else if(d_angle>20) res=1.5;
    else if(d_angle>10) res=2.0; 

    return res;
}

//转运车控制
void TPathTrack::PubCarCtr(data_comm::car_ctr ctr)
{
    if (ctr.turnmode == 0 || ctr.turnmode == 3)
    {
        static data_comm::car_ctr last_ctr;
        float dvel = ctr.speed - last_ctr.speed;
        float max_dvel = 0.05;
        if (dvel > max_dvel)
            ctr.speed = last_ctr.speed + max_dvel;
        else if (dvel < -max_dvel)
            ctr.speed = last_ctr.speed - max_dvel;
        last_ctr = ctr;

        if (ctr.turnmode == 0)  ctr.angle += angle_offset;
    } 
    // printf("ctr.angle=%.2f\n", ctr.angle);

    bool paw_working=(paw_state!="paw_idle" && paw_state!="baojia_done" && paw_state!="wait_for_paw_drop");
    ctr.pawmode=paw_working;

    carctr_pub.publish(ctr);
}

void TPathTrack::BodyFrontErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(move_dir=="front")  body_check_err=msg->data[0];
    // ROS_INFO("1=%.2f\n", bodvoid CheckObsSpeedLimit();y_check_err);
}
void TPathTrack::BodyBackErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(move_dir=="back")  body_check_err=msg->data[0];
    // ROS_INFO("1=%.2f\n", bodvoid CheckObsSpeedLimit();y_check_err);
}

void TPathTrack::WheelErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_check_err=999;
    // 使用前进方向的轮子误差
    if(move_dir=="front" && fabs(msg->data[1])<0.6)  wheel_check_err=msg->data[1];   
    else if(move_dir=="back" && fabs(msg->data[0])<0.6)  
    {
        // wheel_check_err=(msg->data[0]+msg->data[1])*0.5;  
        wheel_check_err=msg->data[0];
        // ROS_INFO("back %.2f  front %.2f",msg->data[0],msg->data[1]);
    }
}

void TPathTrack::Front_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=2)
    {
        front_middle_speedlimit=msg->data[0];
        front_side_speedlimit=msg->data[1];
    }
}

void TPathTrack::Back_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=2)
    {
        back_middle_speedlimit=msg->data[0];
        back_side_speedlimit=msg->data[1];
    }
}
void TPathTrack::Left_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=1)
    {
        left_speedlimit=msg->data[0];
    }
}

void TPathTrack::Right_ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=1)
    {
        right_speedlimit=msg->data[0];
    }
}

void TPathTrack::WorkSpeedlimitCallback(const std_msgs::Float64::ConstPtr &msg)
{
    work_speedlimit=msg->data;
}

void TPathTrack::FrontObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=3)
    {
        front_car_distance=msg->data[2];
    }
}

void TPathTrack::BackObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=3)
    {
        back_car_distance=msg->data[2];
    }
}

void TPathTrack::InsideObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    if(msg->data.size()>=2)
    {
        inside_front_dis=msg->data[0];
        inside_back_dis=msg->data[1];
    }
}

void TPathTrack::AgvInCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (msg->data.size()==2)
    {
        agv_in_flag = msg->data[0]; // agv_in_flag
    }
}

//  接收任务指令
void TPathTrack::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    if (msg->cmd.find("task") != string::npos)
    {
        cur_task = *msg;
        iterative_ctr_flag=0;
        // printf("aaaacmd=%s\n",cur_task.cmd.c_str());
        // self_turn_ctr = turnmode_ctr = 0;   //  退出自转和模式切换循环 20231213
    }
}

void TPathTrack::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length = msg->data;
}

void TPathTrack::PassedPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    passed_path_length = msg->data;
}    

// 接收局部路径 获取预瞄点
void TPathTrack::LocalPathCallback(const nav_msgs::Path::ConstPtr &msg) 
{
    localpath = *msg;
    // ROS_INFO("%d\n", localpath.poses.size());

    CheckSpeedLimit();

    aimpoint.point.x = aimpoint.point.y = 0;    
    aimpoint.header = msg->header;
    aimpoint.header.stamp = ros::Time();
    if (localpath.poses.size() > 1)
    {
        aimpoint.point = (localpath.poses.end() - 1)->pose.position;
    }
    else
    {
        aimpoint.header.frame_id = "";
        localpath.poses.clear();
        self_turn_ctr = turnmode_ctr = 0;
        return;
    }

    float dd = 0;
    for (auto it = localpath.poses.begin(); it != localpath.poses.end() - 2; ++it) // 寻找预瞄点
    {
        float ds = GetDistanceXY(it->pose.position, (it + 1)->pose.position);
        dd += ds;
        if (dd >= aim_range)
        {
            aimpoint.point = it->pose.position; // float wheel_err_front_external = 999, wheel_err_rear_external = 999;
            break;
        }
    }

    if (dd < aim_range)
    {
        // geometry_msgs::PoseStamped target_			char xx=0xff;pose = *(localpath.poses.end() - 2);
        float L1 = aim_range - dd + 0.05; //  最后引导距离
        geometry_msgs::PoseStamped p1 = *(localpath.poses.end() - 1);
        geometry_msgs::PoseStamped p2 = *(localpath.poses.end() - 2);
        p1.pose.orientation = GetQuaternionMsgByPoints(p2.pose.position, p1.pose.position);
        geometry_msgs::PoseStamped p = GetExtendPoseByPose(p1, L1);
        aimpoint.point = p.pose.position;
    }

    ref_speed = localpath.poses[0].pose.position.z; //  速度为首点Z数据
    if (ref_speed > speedlimit)  ref_speed = speedlimit;

    float speedlimit_curve=SpeedLimitByCurve(12);
    if(ref_speed>speedlimit_curve)  ref_speed=speedlimit_curve;

    // printf("vel0=%.2f\n", localpath.poses[0].pose.positio    
    // printf("vel1=%.2f\n", localpath.poses[1].pose.position.z);
}

//  接收车体状态信息
void TPathTrack::CarStateCallback(const data_comm::car_state::ConstPtr &msg) 
{
    cur_carstate = *msg;
}

//  自身转动控制   
//self_turn_ctr=0,1,2,3,4,5
int TPathTrack::SelfTurnCtr() 
{        
    int set_movemode = abs(CheckMoveMode());
    bool is_hengyi_ = (set_movemode == 3);
    static float last_angle_err = 0;
    last_angle_err = track_angle_err;
    GetAimAngleErr(is_hengyi_);
    // printf("track_angle_err=%.2f\n",track_angle_err);

    static TTimer turn_tmr;
    float selfturn_angle_err=60;
    nh_local->getParam("selfturn_angle_err", selfturn_angle_err);
        
    selfturn_disable=(lane_type_init== 23||lane_type_init==24||lane_type_init==25);  // 堆场自转保护
    selfturn_disable=selfturn_disable || (!cur_carstate.holdcar && agv_in_flag);
    // printf("selfturn_disable=%d\n",selfturn_disable);
    // selfturn_disable=false;

    //self_turn_ctr=0,1，2,3    自身转向控制
    //1:停止运动
    if (self_turn_ctr == 0 && fabs(track_angle_err) > selfturn_angle_err && speedlimit > 0.1 )
        self_turn_ctr = 1, turn_tmr.Clear();

    // if(self_turn_ctr) ROS_INFO("%.1f  %d\n", track_angle_err, self_turn_ctr);

    if (self_turn_ctr == 0)  return 0;
    turnmode_ctr = 0;
 
    // data_comm::car_ctr car_ctr;
    car_ctr.workmode = 3;
    car_ctr.speed = car_ctr.angle = 0;
    if (self_turn_ctr == 1) // 停止运动
    {
        car_ctr.turnmode = cur_carstate.turnmode;
        if (turn_tmr.GetValue() > 0.2 && fabs(cur_carstate.speed[0]) < 0.01)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 2 && !selfturn_disable) // 设置自转动
    {
        car_ctr.turnmode = 2;
        if (turn_tmr.GetValue() > 0.2 && cur_carstate.turnmode == 2)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 3) //  转动至角度误差小于阈值
    {
        float max_err = 0.1;      // 最小误差角（由于线性状态机的继续前进的判断有误差变号条件，不需要依靠较大的最小误差保证状态机前进）
        float slow_down_err = 30; // 开始减速的误差角

        // 最大旋转速度             //////////////
        float turn_speed_min = 0.03; // 最小旋转速度
        car_ctr.turnmode = 2;        // 设置自传模式

        float err_abs = fabs(track_angle_err); // 误差的绝对值
        // ROS_INFO("err_angle=%.2f", track_angle_err);
        if (err_abs > max_err && err_abs < slow_down_err) // 在开始减速的误差角度和最小误差角之间，速度线性地由最大转向速度递减为最小转向速度
            car_ctr.speed = turn_speed_min + (err_abs - max_err) / (slow_down_err - max_err) * (turn_speed_max - turn_speed_min);
        else if (err_abs > slow_down_err) // 在开始减速误差角之外采用最大旋转速度
            car_ctr.speed = turn_speed_max;
        else if (err_abs <= max_err || last_angle_err * track_angle_err < 0)
        {    
            car_ctr.speed = 0;
            self_turn_ctr++, turn_tmr.Clear();
        }    
       //超速状况
        if (car_ctr.speed > speedlimit)  car_ctr.speed = speedlimit;

        if (track_angle_err < 0) // 如果是反转加负号
            car_ctr.speed *= -1;
        if (ref_speed < 0)
            car_ctr.speed *= -1;

        if(!run_enable)  car_ctr.speed=0;       

        // ROS_INFO("err=%.1f  self_turn_ctr=%d\n", track_angle_err, self_turn_ctr);    
    }
    else if (self_turn_ctr == 4) // stop turn
    {
        car_ctr.turnmode = 2;
        if (turn_tmr.GetValue() > 0.5)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 5) // 设置为阿克曼转向
    {
        car_ctr.turnmode = set_movemode;
        if (turn_tmr.GetValue() > 2 && car_ctr.turnmode == cur_carstate.turnmode)
            self_turn_ctr = 0;
    }

    // printf("%d\n", car_ctr.turnmode);
    PubCarCtr(car_ctr);

    return self_turn_ctr;
}


//  根据速度的大小，规划预瞄点和转向控制系数
void TPathTrack::UpdateCtrParam(float vel) 
{
    vel = fabs(vel);
    if (vel > 5 )  // speedlimit < 1)
        aim_range = 20, steering_property = 0.6;
    else if (vel > 4.7)
        aim_range = 15, steering_property = 0.7;
    else if (vel > 3.6)
        aim_range = 15, steering_property = 0.8;
    else if (vel > 2.8)
        aim_range = 15, steering_property = 1.0;
    else if (vel > 1.5)
        aim_range = 10, steering_property = 1.2;
    else if (vel > 0.9)
        aim_range = 4, steering_property = 2;
    else 
        aim_range = 3, steering_property = 4;

    if (cur_carstate.turnmode == 2)  aim_range=5;
    // printf("speedlimit=%.2f\n",speedlimit);

    //  预瞄距离滤波
    static TDataFilter f_aimrange(40);
    aim_range=f_aimrange.GetValue(aim_range);
}   

//  根据横移状态和运动速度判断跟踪角度误差
void TPathTrack::GetAimAngleErr(bool is_hengyi_) 
{
    track_angle_err = 0;
    if (aimpoint.header.frame_id == "" || localpath.poses.size() < 1)
    {
        return;
    }
    geometry_msgs::PointStamped dst_p;
    transformPoint("base_link", aimpoint, dst_p, "");

    if (ref_speed >= 0 && !is_hengyi_)
    {
        track_angle_err = atan2(dst_p.point.y, dst_p.point.x);
    }
    else if (ref_speed < 0 && !is_hengyi_)
        track_angle_err = atan2(dst_p.point.y, -dst_p.point.x);
    else if (ref_speed >= 0 && is_hengyi_) // left
        track_angle_err = -atan2(dst_p.point.x, dst_p.point.y);
    else if (ref_speed < 0 && is_hengyi_) // right
        track_angle_err = -atan2(dst_p.point.x, -dst_p.point.y);

    track_angle_err *= 180 / M_PI;

    // printf("hengyi=%d vel=%.2f angle=%.2f\n", is_hengyi_, ref_speed, track_angle_err);

    orien_angle_err = 0;
    geometry_msgs::PoseStamped dst_orien;
    transformPose("base_link", localpath.poses[0], dst_orien, "");
    orien_angle_err = GetYawFromPose(dst_orien) * 180 / M_PI;
}
//  根据预瞄点判断是否横移，右移或左移会使返回参数res改变
int TPathTrack::CheckMoveMode()
{
    int res = 0; // return res;

    if (localpath.poses.size()>1) //  根据预瞄点判断是否横移
    {
        geometry_msgs::PoseStamped p1 = localpath.poses[0];
        geometry_msgs::PoseStamped p2 = localpath.poses[1];
        float pose_angle = GetYawFromPose(p1);
        float path_angle = GetAngleByPoints(p1.pose.position, p2.pose.position);
        float anglex = (path_angle - pose_angle) * 180 / M_PI;

        // printf("%.2f %.2f %.2f\n", pose_angle * 180 / M_PI, path_angle * 180 / M_PI,  anglex);

        if (anglex > 180)
            anglex -= 360;
        else if (anglex < -180)
            anglex += 360;

        // printf("%.1f\n", anglex);

        //  return check
        //后移
        if (fabs(anglex) > 160)    //  move back
            ref_speed = -fabs(ref_speed),  move_dir="back";  
        //前移 
        else if (abs(anglex) < 20)  //  move front
            ref_speed = fabs(ref_speed),  move_dir="front";

        //  hengyi check
        //右移
        if (fabs(anglex + 90) < 20)
            res = -3, ref_speed = -fabs(ref_speed), move_dir="right"; //  right move
        //左移
        else if (fabs(anglex - 90) < 20)
            res = 3, ref_speed = fabs(ref_speed), move_dir="left"; // left move
    }

    return res;
}

// 切换turnmode，
//turnmode_ctr =0、1、2
int TPathTrack::MoveModeCtr() 
{    
    ////  CheckMoveMode()根据预瞄点判断是否横移，右移或左移会使返回参数res改变
    int set_turnmode = abs(CheckMoveMode());
    
    // printf("hengyi=%d\n",set_turnmode);

    static TTimer ctr_tmr;
    
    // ROS_INFO("1 turnmode_ctr=%d %d %d\n", turnmode_ctr, set_turnmode, cur_carstate.turnmode);
    if (turnmode_ctr == 0 && self_turn_ctr == 0 && set_turnmode != cur_carstate.turnmode && localpath.poses.size()>1)
        turnmode_ctr = 1, ctr_tmr.Clear();

    // ROS_INFO("%d %d %d %d\n", set_turnmode, cur_carstate.turnmode, turnmode_ctr, self_turn_ctr);
    if (turnmode_ctr == 0)  return 0;

    // ROS_INFO("2 turnmode_ctr=%d %.1f %.2f\n", turnmode_ctr, ctr_tmr.GetValue(), cur_carstate.speed[0]);
 
    // data_comm::car_ctr car_ctr;
    car_ctr.workmode = 3;
    car_ctr.speed = car_ctr.angle = 0;

    if (turnmode_ctr == 1) // 停止运动
    {
        car_ctr.turnmode = cur_carstate.turnmode;
        if (ctr_tmr.GetValue() > 0.2 && fabs(cur_carstate.speed[0]) < 0.01)
            turnmode_ctr++, ctr_tmr.Clear(); // 等待停止
    }
    if (turnmode_ctr == 2) // 设置turnmode
    {
        car_ctr.turnmode = set_turnmode;
        // printf("turnmode: state=%d  set=%d\n", cur_carstate.turnmode, car_ctr.turnmode);
        if (set_turnmode == cur_carstate.turnmode)
            turnmode_ctr = 0;
    }
    PubCarCtr(car_ctr);
    // carctr_pub.publish(car_ctr);
    return turnmode_ctr;
}

//  停车控制
int TPathTrack::StopCtr() 
{
    int stop_flag = StopCheck();
    if (stop_flag)
    {
        //  设置充电动作
        if(cur_task.cmd=="charge task" && stop_str== "no path")  car_ctr.workmode=4;
        else  car_ctr.workmode = 2;

        car_ctr.turnmode = cur_carstate.turnmode;
        car_ctr.speed = car_ctr.angle = 0;
        PubCarCtr(car_ctr);
        // printf("stop_flag=%d ----car_ctr: %d\n", stop_flag, car_ctr.workmode);
        // carctr_pub.publish(car_ctr);
    }

    return stop_flag;
}

int TPathTrack::FaultCheck()
{
    static TTimer tmr;
    string errstr="";
    nh->getParam("/iot_comm/err", errstr);
    if(errstr=="") tmr.Clear();  
    
    if(tmr.GetValue()>3)  return 1;
    else return 0;
}

//  综合判断是否停车  1 传感器与节点故障 2 局部路径 3 运动使能 4 抱夹作业 5 雷达定位
int TPathTrack::StopCheck() 
{
    stop_str="";
    if(FaultCheck())
    {
        stop_str = "fault stop";
    }

    // ROS_INFO("%d", localpath.poses.size());
    else if (localpath.poses.size() == 0) //  无有cur_task.final_path效路径
    {
        stop_str = "no path";
    }

    else if (!run_enable) //  自动导航断使能
        stop_str = "run disable";

    // 夹爪作业
    else if (paw_state == "car_stop" || paw_state == "length_change_start" || 
        paw_state == "length_change_done" || paw_state == "baojia_start" || paw_state == "car_dropping" ||
        (paw_state == "paw_dropping" && agv_in_flag)) // || paw_state_msg.paw_state==2)  //  夹爪在张合过程中不能动
        stop_str = "paw working";

    else if (cur_task.final_path && agv_in_flag && cur_task.cmd=="pick task" && fabs(wheel_check_err)>1)
        stop_str = "wheel check err";

    else if (cur_task.final_path && car_distance>1 && !agv_in_flag && cur_task.cmd=="pick task" && remain_path_length<3)
        stop_str = "pick empty";    

    else if (cur_carstate.workmode == 1) // 急停按钮
        stop_str = "emc stop";

    else if (cur_carstate.ctrmode == 0) //  手动按钮
        stop_str = "manual";

    else if (speedlimit<0.01 && iterative_ctr_flag==0)
    {
        if(mistop_speedlimit<0.01)   stop_str="mi_stop";
        else if(obstacle_speedlimit<0.01)   stop_str="obstacle_stop";
    }

    // if(nodecheck->Find("track_err_rate")->value>10)  stop_str="track_err";

    else if(self_turn_ctr>0 && selfturn_disable)  stop_str = "selfturn err";

    if (stop_str != "")  return 1;
    else  return 0;
}

/// @brief 
void TPathTrack::CheckObsSpeedLimit()
{
    obstacle_speedlimit = 999;
    if(!obs_stop_enable)  return;

    // printf("move_dir=%s\n",move_dir.c_str());
    if (cur_carstate.turnmode == 2) //  自转运动
    {
        obstacle_speedlimit = min(right_speedlimit,left_speedlimit);
        float front_speedlimit=min(front_middle_speedlimit,front_side_speedlimit);
        float back_speedlimit=min(back_middle_speedlimit,back_side_speedlimit);
        obstacle_speedlimit = min(obstacle_speedlimit,front_speedlimit);
        obstacle_speedlimit = min(obstacle_speedlimit,back_speedlimit);
        // printf("selfturn_obstacle_speedlimit=%.3f\n", obstacle_speedlimit);
    }
    else if (move_dir == "front" || move_dir=="back") //  前向运动
    {
        float middle_speedlimit, side_speedlimit;
        if(move_dir=="front")  middle_speedlimit=front_middle_speedlimit,  side_speedlimit=front_side_speedlimit;
        else middle_speedlimit=back_middle_speedlimit,  side_speedlimit=back_side_speedlimit;

        if(iterative_ctr_flag>0 && car_ctr.speed>0)  middle_speedlimit=front_middle_speedlimit,  side_speedlimit=front_side_speedlimit;
        else if(iterative_ctr_flag>0 && car_ctr.speed<0)   middle_speedlimit=back_middle_speedlimit,  side_speedlimit=back_side_speedlimit;
        // printf("%s middle=%.2f\n",move_dir.c_str(), middle_speedlimit);
        // printf("remain_path=%.2f\n",remain_path_length);

        float final_path_len=12, final_car_dis=6;
        if(cur_task.subcmd=="2") final_path_len=7;  //立体车库有坡度，距离短点
        bool field_flag = (lane_type_init==23 || lane_type_init==24 || lane_type_init==25 || lane_type_init==31);
        bool is_final_path=(remain_path_length < final_path_len || (car_distance<final_car_dis && field_flag)) && cur_task.final_path;
            
        //  取车最后路程, AGV无车,速度受限边沿障碍物
        if(cur_task.cmd=="pick task" && is_final_path) 
        {
            // 检测到轮胎后，取消避障
            if(fabs(wheel_check_err)>1)  obstacle_speedlimit=side_speedlimit;
            else  obstacle_speedlimit=999;
            // printf("speedlimit=%.2f \n",obstacle_speedlimit);
            static TTimer tmr;
            if(stop_str!="obstacle_stop" && stop_str!="wheel check err")  tmr.Clear();
            else if(tmr.GetValue()>1 && iterative_ctr_enable && paw_state_msg.paw_state==0)  iterative_ctr_flag=1; 
        }
        //  密停放车最后路程, AGV有车, 速度受限密停距离和边沿障碍物    
        else if(cur_task.cmd=="release task" && cur_task.subcmd=="1" && is_final_path)
            obstacle_speedlimit=min(mistop_speedlimit, side_speedlimit);
        else 
            obstacle_speedlimit=min(middle_speedlimit, side_speedlimit);      

        if(fabs(car_ctr.angle)>20)  //  大转向考虑左右障碍物
        {
            obstacle_speedlimit=min(obstacle_speedlimit, left_speedlimit);
            obstacle_speedlimit=min(obstacle_speedlimit, right_speedlimit);
        }    
    }
    else if (move_dir == "left") //  左侧横移运动
    {
        obstacle_speedlimit = left_speedlimit;
    }
    else if (move_dir == "right") //  右侧横移运动
    {
        obstacle_speedlimit = right_speedlimit;
    }

    // printf("obs=%.2f\n",obstacle_speedlimit);
    if(obstacle_speedlimit<0.04) obstacle_speedlimit=0;
}

void TPathTrack::CheckSpeedLimit()
{
    nh_local->getParam("/mi_stop/speedlimit", mistop_speedlimit);
    
    // printf("speedlimit=%.2f\n", mistop_speedlimit);
    speedlimit = 999;
    if (speedlimit >= work_speedlimit)  speedlimit = work_speedlimit;
    if (speedlimit >= mistop_speedlimit)  speedlimit = mistop_speedlimit;

    // ROS_INFO("mode=%d SPD=%.2f",car_ctr.turnmode, car_ctr.speed);
    CheckObsSpeedLimit();   
    if (speedlimit >= obstacle_speedlimit)  
    {
        speedlimit = obstacle_speedlimit;
        // printf("obs=%.2f\n",obstacle_speedlimit);
        std_msgs::Float64 obs_dis;
        obs_dis.data=obstacle_speedlimit; 
        obs_speedlimit_pub.publish(obs_dis);
    }
    if (speedlimit < 0.001)  speedlimit = 0.001;
    // printf("speedlimit=%.2f obs_speedlimit=%.2f  work_speedlimit=%.2f\n", speedlimit, obstacle_speedlimit, work_speedlimit);
}


// 纯轨迹跟踪控制
void TPathTrack::PurePursuit(data_comm::car_ctr &car_ctr)
{
    car_ctr.turnmode = cur_carstate.turnmode;
    car_ctr.workmode = 3;

    car_ctr.speed = ref_speed;
    // if (cur_task.cmd == "pick task" && remain_path_length < 7)
    //     car_ctr.angle = steering_property * (track_angle_err + orien_angle_err);
    // else
    car_ctr.angle = steering_property*track_angle_err;
    // printf("car_ctr=%.2f\n",car_ctr.angle);
}


// 纠偏控制（重要）
//实际值与期望值存在误差，对控制进行补偿
void TPathTrack::DeviationControl(data_comm::car_ctr &car_ctr)
{
    float wheel_err = 999;

    static TTimer tmr;
    if(iterative_ctr_flag==0)  tmr.Clear();
    else if(iterative_ctr_flag==1)   //  倒车运动6s，留出纠偏距离
    {
        if(paw_state == "length_change_start") iterative_ctr_flag=0; 

        if(move_dir=="front")   car_ctr.speed = -0.5;
        else car_ctr.speed = 0.5;
        if(tmr.GetValue()>6)  iterative_ctr_flag++, tmr.Clear();
    }
    else if(iterative_ctr_flag==2)   //  停车2s，标志位清除
    {
        car_ctr.speed=0;
        if(tmr.GetValue()>2)  iterative_ctr_flag=0;
    }
    
    float final_path_len=20, final_car_dis=6;
    bool field_flag = (lane_type_init==23 || lane_type_init==24 || lane_type_init==25 || lane_type_init==31);
    bool is_final_path=(remain_path_length<final_path_len || (car_distance<final_car_dis && field_flag)) && cur_task.final_path;
        
    // 取车纠偏
    // if (cur_task.cmd == "pick task" && remain_path_length < 20 && (car_distance<5 || agv_in_flag) && cur_task.final_path && iterative_ctr_flag!=1)
    if (cur_task.cmd == "pick task" && is_final_path && iterative_ctr_flag!=1)
    {
        if (fabs(wheel_check_err)<1 && agv_in_flag)  wheel_err = wheel_check_err;
        else if (fabs(body_check_err) < 1)  wheel_err = body_check_err;
        // ROS_INFO("%.2f  %.2f\n", wheel_check_err, body_check_err);
    }
    // 密停区放车纠偏
    // else if (cur_task.cmd == "release task" && cur_task.subcmd=="1" && remain_path_length<20 && car_distance<5 && cur_task.final_path && iterative_ctr_flag!=1)
    // {
    //     wheel_err = body_check_err;
    //     // ROS_INFO("%.2f\n", wheel_err);
    // }
    // 迭代取车退回过程 或者 AGV退出商品车的时候 不纠偏
    else if((agv_in_flag && paw_state_msg.paw_state==0 && passed_path_length<6) || iterative_ctr_flag==1)  //AGV退车时(爪收回)不纠偏
        wheel_err=0;

    // 纠偏控制
    if (fabs(wheel_err) < 2 && laser_work_enable)
    {
        float max_value=6;  //  最大值 增益
        // if(left_speedlimit<0.1 || right_speedlimit<0.1)   //  左右有很近障碍物时，控制参数要小
        // {
        //     max_value=4, kp=10;
        // }
        
        car_ctr.angle = DeviationControl_Kp*wheel_err;   //  比例控制
        if (car_ctr.angle < -max_value)   car_ctr.angle = -max_value;
        else if (car_ctr.angle > max_value)    car_ctr.angle = max_value;
        // ROS_INFO("wheel_err=%.2f\n", wheel_err);
    }
}

//显示跟踪状态，每dt时间显示一次
void TPathTrack::TS_Publish()
{
    string paw_state_str;
    //夹爪状态
    nh->getParam("/pawcontrol/paw_state", paw_state_str);

    cur_ts.timestamp=int64_t((ros::Time::now().toSec())*1000.0);  //时间戳转化为浮点
    cur_ts.aim_range=aim_range;   //预瞄点距离
    cur_ts.steering_property=steering_property;   //转角系数
    cur_ts.self_turn_ctr=self_turn_ctr;   //转向控制
    cur_ts.turnmode_ctr=turnmode_ctr;   
    cur_ts.track_angle_err=track_angle_err;  //跟踪角度误差
    cur_ts.track_dis_err=CheckTrackErr();   //跟踪误差大小
    // cur_ts.car_state=cur_carstate;
    // cur_ts.car_ctr=car_ctr;
    // cur_ts.cur_task=cur_task;
    cur_ts.stopReason=stop_str;
    track_state_pub.publish(cur_ts);   //发布以上跟踪状态信息


    static TTimer tmr;
    float dt=2;
    if(cur_task.cmd!="")  dt=0.05;
     
     //每dt时间，显示一次数据
    if(tmr.GetValue()>dt)   //计时器
    {
        ROS_INFO("(x=%.2f,y=%.2f,a=%.2f)(selftu=%d,turnmo=%d)\
(anger=%.1f,diser=%.1f) (task=%s,subcmd=%s,final=%d rp=%.2f) (z.work=%d,z.turn=%d,z.spd=%.1f hold=%d paw=%d %d)\ 
(c.work=%d,c.turn=%d c.spd=%.1f,c.ang=%.1f) stop=%s paw=%s st=%d",GPS_msg.x,GPS_msg.y,GPS_msg.z,\
cur_ts.self_turn_ctr,cur_ts.turnmode_ctr,\
cur_ts.track_angle_err,cur_ts.track_dis_err,\
cur_task.cmd.c_str(),cur_task.subcmd.c_str(),cur_task.final_path,remain_path_length,\
cur_carstate.workmode,cur_carstate.turnmode,cur_carstate.speed[0],cur_carstate.holdcar,paw_state_msg.carhold_state,paw_state_msg.paw_state,\
car_ctr.workmode,car_ctr.turnmode,car_ctr.speed,car_ctr.angle,\
cur_ts.stopReason.c_str(),paw_state_str.c_str(),agv_state.state_flow);
        tmr.Clear();
    }
}

void TPathTrack::Run() //  主运行函数
{
    nodecheck->Find("node_rate")->Beat();     //mycounter 计数器
     
    //跟踪误差是否小于1
    if(CheckTrackErr()<1)  nodecheck->Find("track_err_rate")->Beat();
   
    ////显示跟踪状态，每dt时间显示一次
    if(pub_enable) TS_Publish(); //20231120
                                 
    CheckMoveMode();   ////  根据预瞄点判断是否横移  //右移 res=-3//左移 res=3
                   
     //move_dir = "front"、"back"、right、left 移动方向
    if(move_dir == "front") 
    {
        car_distance=front_car_distance;
        if(inside_front_dis<5)  car_distance+=inside_front_dis;
    }
    else if(move_dir == "back") 
    {
        car_distance=back_car_distance;
        if(inside_back_dis<5)  car_distance+=inside_back_dis;
    }
     
    std_msgs::Float32MultiArray car_dis_inout_msg;
    car_dis_inout_msg.data.push_back(car_distance);   //???
    //发布话题
    car_dis_pub.publish(car_dis_inout_msg);  

    std_msgs::String str_msg;
    str_msg.data=move_dir;   //move_dir = "front"、"back"、right、left 移动方向
    //发布运动方向
    move_dir_pub.publish(str_msg);
    str_msg.data=stop_str;
    //发布话题
    stop_reason_pub.publish(str_msg);
    std_msgs::Int32 int_msg;
    //发布话题
    int_msg.data=self_turn_ctr;
    selfturn_ctr_pub.publish(int_msg);

    // data_comm::car_ctr car_ctr;

    if (!test_flag)   //  正常模式
    {
        nh_local->getParam("run_enable", run_enable);
        nh_local->getParam("pub_enable", pub_enable); 
        nh_local->getParam("iterative_ctr_enable", iterative_ctr_enable);
        nh_local->getParam("obs_stop_enable", obs_stop_enable);
        nh_local->getParam("turn_speed_max", turn_speed_max);
        nh_local->getParam("/pawcontrol/paw_state", paw_state);
        nh_local->getParam("/local_path_plan/lane_type", lane_type_init);
        
        // ROS_INFO("paw_state=%s ", paw_state.c_str()); 

        / / 根据速度规划预瞄点和转向控制系数
        UpdateCtrParam(ref_speed);  
        if (localpath.poses.size())  aimpoint_pub.publish(aimpoint);   //发布点云信息
        
        //车体自身转动控制
        //返回值： self_turn_ctr=0,1,2,3,4,5
        if (SelfTurnCtr())    
        {
            stop_str="self turning";    //self_turn_ctr不等于零，
            return;
        }

       //改变运动模式
       //turnmode_ctr 转向模式控制
       //turnmode_ctr=0,1,2,3
        if (cur_task.cmd.find("task") != string::npos && MoveModeCtr() && stop_str!="no path")
        {
            stop_str="movemode changing";
            return;
        }
        
        if (StopCtr())  return;
        n_save=0;

        stop_str="";
        PurePursuit(car_ctr);      // 纯跟踪控制
        DeviationControl(car_ctr); //  纠偏控制  放在最后，优先级最高
    }
    else  // 开环测试模式
    {
        car_ctr.turnmode = 0;
        car_ctr.workmode = 3;
        car_ctr.angle = 0;
        car_ctr.speed = test_speed;
    }
    
    PubCarCtr(car_ctr);

    std_msgs::Float64 trackerr_msg;
    trackerr_msg.data = CheckTrackErr();   //检测跟踪误差
    track_err_pub.publish(trackerr_msg);   //发布跟踪误差
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathtrack");    //初始化节点
    TPathTrack pathtrack;

    ros::Rate rate(20);     //定义频率
    while (ros::ok())
    {
        pathtrack.Run();   //  主运行函数
        ros::spinOnce();
        rate.sleep();
        
    }

    return 0;
}
