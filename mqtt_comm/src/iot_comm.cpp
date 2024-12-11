#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <common/public.h>
#include <common/mydisplay.h>
#include "geometry_msgs/PoseStamped.h"
// #include "mqtt_comm/controls.h"

#include "mqtt_comm/iot_controls.h"
#include "mqtt_comm/iot_taskpath.h"
#include "mqtt_comm/iot_tasks.h"
#include "mqtt_comm/fault_info.h"
#include "mqtt_comm/fault_info_array.h"
#include "mqtt_comm/resp_iot.h"
#include "mqtt_comm/path_point.h"
#include "mqtt_comm/iot_path_point.h"
#include "mqtt_comm/task.h"
#include "mqtt_comm/cmd_resp.h"
#include "mqtt_comm/charge_station_notify.h"
// #include "mqtt_comm/resp_agvstate.h"
// #include "mqtt_comm/resp_task.h"
// #include "mqtt_comm/resp_video.h"
// #include "mqtt_comm/resp_ctrl.h"
#include "mqtt_comm/network_heartbeat.h"

#include <gps/MyGPS_msg.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#include <data_comm/paw_state.h>
#include <std_msgs/Int32MultiArray.h>

using namespace std;

struct sensor_item
{
    string nodename;
    string paramname;
    float rate_min;
    mqtt_comm::fault_info fault;
};

struct node_item  
{
    int id;
    string nodename;
    float rate_min;
    mqtt_comm::fault_info fault;
};


class Iot_comm
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher resp_iot_tasks_pub, resp_iot_taskpath_pub, resp_iot_controls_pub,resp_iot_charge_pub;
    ros::Publisher resp_iot_pub;
    ros::Publisher task_pub;
    // ros::Publisher sensor_pub;
    ros::Subscriber iot_tasks_sub,iot_taskpath_sub,iot_controls_sub,iot_charge_sub,paw_state_sub,carstate_sub,carctr_sub,remainpath_sub;
    ros::Subscriber movedir_sub, iot_heart_sub,obs_dis_sub,agv_in_sub,localpath_sub, selfturn_sub, mi_stop_sub;

    double utm_x_zero = 0, utm_y_zero = 0;
    // mqtt_comm::controls iot_control;
    mqtt_comm::task task_msg;
    mqtt_comm::task task_msg_old;
    data_comm::paw_state paw_state;
    data_comm::car_state car_state;
    data_comm::car_ctr car_ctr;

    TNodeCheck *nodecheck;
    mqtt_comm::resp_iot agvstate_msg; // printf("\n");sensor_array
    float remain_path_length = 999;

    mqtt_comm::fault_info_array fault_array;
    vector<sensor_item> sensor_array;
    vector<node_item> node_array;

    bool run_enable = true;

    bool stop_flag= false;
    int first_carstate = -1;
    //bool obs_speedlimit = false;
    float obs_speedlimit = 3;
    bool selfturn_Err = false;
    
    TTimer task_tmr, path_tmr, selfturn_tmr;
    bool turn_end_flag=false;
    string paw_state_str="";
    bool agv_in_flag=false;
    nav_msgs::Path localpath;
    float localpath_dis;
    int selfturn_ctr=0, mi_stop_state=0;
    bool new_task_flag=true;
    string move_dir="";

public:
    Iot_comm()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();

        nodecheck = new TNodeCheck(nh_local, "iotheart_rate");

        SensorFaultInit();

        // 接收IOT消息(任务and路径and控制and充电)
        iot_tasks_sub = nh->subscribe<mqtt_comm::iot_tasks>("/iot_task",10,&Iot_comm::tasksCallback,this);
        iot_taskpath_sub = nh->subscribe<mqtt_comm::iot_taskpath>("/iot_path", 0,&Iot_comm::taskpathCallback,this);
        iot_controls_sub = nh->subscribe<mqtt_comm::iot_controls>("/iot_control",10,&Iot_comm::controlsCallback,this);
        iot_charge_sub = nh->subscribe<mqtt_comm::charge_station_notify>("/charge_station_notify",10,&Iot_comm::chargeCallback,this);
        iot_heart_sub= nh->subscribe<mqtt_comm::network_heartbeat>("/mqtt_comm/network_heartbeat",10,&Iot_comm::heartCallback,this);
        // // 接收车端消息
        paw_state_sub = nh->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, &Iot_comm::PawStateCallback,this);
        carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &Iot_comm::CarStateCallback,this);
        localpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, &Iot_comm::LocalPathCallback, this);
        carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, &Iot_comm::CarCtrCallback,this);
        remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &Iot_comm::RemainPathCallback, this);
        obs_dis_sub = nh->subscribe<std_msgs::Float64>("/obs_speedlimit", 10, &Iot_comm::ObsdisCallback, this);
        agv_in_sub = nh->subscribe<std_msgs::Int32MultiArray>("/cloud_calculation_inside/agv_in_flag", 10, &Iot_comm::AgvInCallback, this);
        selfturn_sub = nh->subscribe<std_msgs::Int32>("/selfturn_ctr", 10, &Iot_comm::SelfTurnCallback, this);
        mi_stop_sub = nh->subscribe<std_msgs::Int32>("/mi_stop/stop_ctr_state", 10, &Iot_comm::MiStopCallback, this);
        movedir_sub = nh->subscribe<std_msgs::String>("/move_dir", 10, &Iot_comm::MoveDirCallback, this);
        
        // 回应IOT消息
        resp_iot_tasks_pub = nh_local->advertise<mqtt_comm::cmd_resp>("/resp_iot_tasks", 10);
        resp_iot_taskpath_pub = nh_local->advertise<mqtt_comm::cmd_resp>("/resp_iot_taskpath", 10);
        resp_iot_controls_pub = nh_local->advertise<mqtt_comm::cmd_resp>("/resp_iot_controls", 10);
        resp_iot_charge_pub = nh_local->advertise<mqtt_comm::cmd_resp>("/charge_station_notify/resp",10);
        resp_iot_pub = nh_local->advertise<mqtt_comm::resp_iot>("/resp_iot", 10);

        // 发布给车端消息
        task_pub = nh_local->advertise<mqtt_comm::task>("/task_cmd", 10);
        nh->setParam("selfturn_err",false);
    };

    void tasksCallback(const mqtt_comm::iot_tasks::ConstPtr &msg);
    void taskpathCallback(const mqtt_comm::iot_taskpath::ConstPtr &msg);
    void controlsCallback(const mqtt_comm::iot_controls::ConstPtr &msg);
    void chargeCallback(const mqtt_comm::charge_station_notify::ConstPtr &msg);
    void PawStateCallback(const data_comm::paw_state::ConstPtr &msg);
    void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
    void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg);
    void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void ObsdisCallback(const std_msgs::Float64::ConstPtr &msg);
    void heartCallback(const mqtt_comm::network_heartbeat::ConstPtr &msg);
    void AgvInCallback(const std_msgs::Int32MultiArray::ConstPtr &msg);
    void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    void SelfTurnCallback(const std_msgs::Int32::ConstPtr &msg);
    void MiStopCallback(const std_msgs::Int32::ConstPtr &msg);
    void MoveDirCallback(const std_msgs::String::ConstPtr &msg);

    void controls_Proc(mqtt_comm::iot_controls m); // 处理控制信号
    void Resp_controls(mqtt_comm::iot_controls m); // 回应控制信号
    void taskpath_Proc(mqtt_comm::iot_taskpath m); // 处理路径指令
    void Resp_taskpath(mqtt_comm::iot_taskpath m); // 回应路径指令
    bool refuse_taskpath(mqtt_comm::iot_taskpath m);  // 拒绝路径
    bool refuse_task(mqtt_comm::iot_tasks t);  // 拒绝任务
    void tasks_Proc(mqtt_comm::iot_tasks m); // 处理任务指令
    void Resp_tasks(mqtt_comm::iot_tasks m); // 回应任务指令
    void charge_Proc(mqtt_comm::charge_station_notify m);
    void Resp_charge(mqtt_comm::charge_station_notify m);
    void PubIOT_AgvState(); //  发布车辆状态
    void heart_Proc(mqtt_comm::network_heartbeat m);
    // void slow_stop();   //缓慢停车
    void GenStopPath(float len);

    bool CheckSensor(sensor_item sensor);
    bool CheckNode(node_item node);
    void SensorFaultInit();
    void UpdateSensorFaultLevel();
    void UpdateFaultInfo();
    void clearTask();
};

void Iot_comm::clearTask()
{
    task_pub.publish(task_msg);
    task_msg.cmd = "";
    task_msg.final_path = 0;
    agvstate_msg.state_flow = 0;
    agvstate_msg.task_id = "";
    agvstate_msg.task_type = 0;
    agvstate_msg.target = "";
    agvstate_msg.sub_target = "";
}

void Iot_comm::GenStopPath(float len)
{
    task_msg.final_path=false;
    task_msg.path.clear();
    task_msg.path.resize(2);

    // 获得当前坐标点
    geometry_msgs::PoseStamped p_base, p_map;
    p_base.header.frame_id="base_link";
    p_base.header.stamp=ros::Time::now();
    p_base.pose.orientation.w=1;
    transformPose("map", p_base, p_map);
    // printf("x=%.2f y=%.2f\n",p_map.pose.position.x,p_map.pose.position.y);

    //  路径第一点为当前点和当前速度
    task_msg.path[0].pointX=p_map.pose.position.x;
    task_msg.path[0].pointY=p_map.pose.position.y;
    task_msg.path[0].pointHA=GetYawFromPose(p_map)*180/M_PI;
    task_msg.path[0].vehSpeed=car_ctr.speed;

    //  根据速度方向获取前方坐标点
    if(car_ctr.speed<0) len=-len;  
    p_map=GetExtendPoseByPose(p_map, len);
    
    //  第二点为缓停点
    task_msg.path[1].pointX=p_map.pose.position.x;
    task_msg.path[1].pointY=p_map.pose.position.y;
    task_msg.path[1].pointHA=GetYawFromPose(p_map)*180/M_PI;
    task_msg.path[1].vehSpeed=0;
    
    task_msg.cmd="move task";
    // task_pub.publish(task_msg); // 给车端发路径指令
} 


bool Iot_comm::CheckSensor(sensor_item sensor)
{
    float value=0;
    nh->getParam(sensor.nodename+"/check/"+sensor.paramname, value);
    return value>=sensor.rate_min;
}

bool Iot_comm::CheckNode(node_item node)
{
    bool res = false;
    float noderate=0;
    static int old_seq[20];
    int seq=0;
    nh->getParam(node.nodename+"/check/seq", seq);
    nh->getParam(node.nodename+"/check/node_rate", noderate); 
    res=old_seq[node.id]!=seq;
    old_seq[node.id] = seq;
    return res && noderate>=node.rate_min;
}

void Iot_comm::UpdateFaultInfo()
{
    nh->getParam("selfturn_err",selfturn_Err);

    fault_array.timestamp = int64_t((ros::Time::now().toSec())*1000.0);
    fault_array.fault_info_data.clear();
    
    for(auto s: sensor_array)  
    {
        if(!CheckSensor(s))  
        {
            fault_array.fault_info_data.push_back(s.fault);
        }
    }

    for(auto n: node_array)
    {
        if(!CheckNode(n))
        {
            fault_array.fault_info_data.push_back(n.fault);
        }  
    }
    if(first_carstate==1)
    {
        mqtt_comm::fault_info fault;
        fault.desc = "ctrmode fault";
        fault.code=0x3001;
        fault.level=1;
        fault_array.fault_info_data.push_back(fault);
    }
    if(obs_speedlimit<2.5)
    {
        mqtt_comm::fault_info fault;
        fault.desc = "obstacle speedlimit";
        fault.code=0x4001;
        fault.level=2;
        fault_array.fault_info_data.push_back(fault);
    }
    if(selfturn_Err==true)
    {
        mqtt_comm::fault_info fault;
        fault.desc = "selfturn false";
        fault.code=0x4002;
        fault.level=2;
        fault_array.fault_info_data.push_back(fault);
    }

    // 底盘故障
    if(car_state.errcode>0)
    {
        mqtt_comm::fault_info fault;
        fault.desc="driver fault";
        int a = (car_state.errcode >> 24) & 0xFF;
        fault.code=car_state.errcode;
        if(a==1 || a==2)  fault.level=1;
        else fault.level=2;
        fault.level=2;
        if(fault.code!=0x4010202)  fault_array.fault_info_data.push_back(fault);
    }

    char errcode[1000]={0}, warncode[1000]={0};
    for(auto it:fault_array.fault_info_data)
    {
        if(it.level==1)  sprintf(errcode,"%s %04x", errcode, it.code);
        else  sprintf(warncode,"%s %04x", warncode, it.code);
    }    
    //if(err!="")  ROS_ERROR("%s", err.c_str());
    nh_local->setParam("err",errcode);
    nh_local->setParam("warn",warncode);

    // ROS_INFO("%d", fault_array.fault_info_data.size());
}

void Iot_comm::UpdateSensorFaultLevel()
{
    for(auto &it:sensor_array)
    {
        if(it.nodename=="/laserscan_check_angle")  //  取车时线雷达报警级别最高
        {
            if(task_msg.cmd=="pick task" || task_msg.cmd=="") it.fault.level=1;
            else it.fault.level=2;
        }
        else if(it.nodename=="/cloud_tf2" || it.nodename=="/cloud_tf4")  // 前向时，前多线雷达报警级别最高
        {
            if(move_dir=="front")  it.fault.level=1;
            else it.fault.level=2;
        }
        else if(it.nodename=="/cloud_tf1" || it.nodename=="/cloud_tf3")  // 后向时，后多线雷达报警级别最高
        {
            if(move_dir=="back")  it.fault.level=1;
            else it.fault.level=2;
        }
    }
}

void Iot_comm:: SensorFaultInit()
{
    sensor_item sensor_data;
    node_item node_data;
    
    // 传感器故障信息
    sensor_data.nodename = "/gps_pro";                     //gps
    sensor_data.paramname = "node_rate";
    sensor_data.rate_min = 15;
    sensor_data.fault.desc = "gps fault";
    sensor_data.fault.code=0x1001;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/can_comm";                    //can
    sensor_data.paramname = "can_rate";
    sensor_data.rate_min = 45;
    sensor_data.fault.desc = "can fault";
    sensor_data.fault.code=0x1002;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/iot_comm";                    //iot
    sensor_data.paramname = "iotheart_rate";
    sensor_data.rate_min = 3;
    sensor_data.fault.desc = "iot fault";
    sensor_data.fault.code=0x1003;
    sensor_data.fault.level=2;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/laserscan_check_angle";       //单线
    sensor_data.paramname = "scan1_rate";
    sensor_data.rate_min = 10;
    sensor_data.fault.desc = "scan1_rb fault";
    sensor_data.fault.code=0x1004;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/laserscan_check_angle";
    sensor_data.paramname = "scan2_rate";
    sensor_data.rate_min = 10;
    sensor_data.fault.desc = "scan2_lf fault";
    sensor_data.fault.code=0x1005;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/laserscan_check_angle";
    sensor_data.paramname = "scan3_rate";
    sensor_data.rate_min = 10;
    sensor_data.fault.desc = "scan3_lb fault";
    sensor_data.fault.code=0x1006;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data); 

    sensor_data.nodename = "/laserscan_check_angle";
    sensor_data.paramname = "scan4_rate";
    sensor_data.rate_min = 10;
    sensor_data.fault.desc = "scan4_rf fault";
    sensor_data.fault.code=0x1007;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);   

    sensor_data.nodename = "/cloud_tf1";                  //多线
    sensor_data.paramname = "rslidar_rate";
    sensor_data.rate_min = 5;
    sensor_data.fault.desc = "rslidar1_rb fault";
    sensor_data.fault.code=0x1008;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/cloud_tf2";                
    sensor_data.paramname = "rslidar_rate";
    sensor_data.rate_min = 5;
    sensor_data.fault.desc = "rslidar2_lf fault";
    sensor_data.fault.code=0x1009;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/cloud_tf3";                
    sensor_data.paramname = "rslidar_rate";
    sensor_data.rate_min = 5;
    sensor_data.fault.desc = "rslidar3_lb fault";
    sensor_data.fault.code=0x100A;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/cloud_tf4";                
    sensor_data.paramname = "rslidar_rate";
    sensor_data.rate_min = 5;
    sensor_data.fault.desc = "rslidar4_rf fault";
    sensor_data.fault.code=0x100B;
    sensor_data.fault.level=1;
    sensor_array.push_back(sensor_data);

    sensor_data.nodename = "/rfid_reader_tcp";           //RFID
    sensor_data.paramname = "rfid_rate";
    sensor_data.rate_min = 0.5;
    sensor_data.fault.desc = "RFID fault";
    sensor_data.fault.code=0x100C;
    sensor_data.fault.level=2;
    sensor_array.push_back(sensor_data);

    // 节点故障信息
    node_data.id=0;
    node_data.nodename="/local_path_plan";
    node_data.fault.desc="local_path_plan fault";
    node_data.rate_min = 10;
    node_data.fault.code=0x2001;
    node_data.fault.level=1;
    node_array.push_back(node_data);

    node_data.id=1;
    node_data.nodename="/pathtrack";
    node_data.fault.desc="pathtrack fault";
    node_data.rate_min = 5;
    node_data.fault.code=0x2002;
    node_data.fault.level=1;
    node_array.push_back(node_data);

    node_data.id=2;
    node_data.nodename="/pawcontrol";
    node_data.fault.desc="paw_control fault";
    node_data.rate_min = 10;
    node_data.fault.code=0x2003;
    node_data.fault.level=1;
    node_array.push_back(node_data);

    node_data.id=3;
    node_data.nodename="/laserscan_check";
    node_data.fault.desc="laserscan_check fault";
    node_data.rate_min = 10;
    node_data.fault.code=0x2004;
    node_data.fault.level=2;
    node_array.push_back(node_data);

    node_data.id=4;
    node_data.nodename="/laserscan_check_angle";
    node_data.fault.desc="laserscan_check_angle fault";
    node_data.rate_min = 10;
    node_data.fault.code=0x2005;
    node_data.fault.level=2;
    node_array.push_back(node_data);

    node_data.id=5;
    node_data.nodename="/cloud_segmentation";
    node_data.fault.desc="cloud_segmentation fault";
    node_data.rate_min = 5;
    node_data.fault.code=0x2006;
    node_data.fault.level=2;
    node_array.push_back(node_data);

    node_data.id=6;
    node_data.nodename="/cloud_calculation_inside";
    node_data.fault.desc="cloud_calculation_inside fault";
    node_data.rate_min = 5;
    node_data.fault.code=0x2007;
    node_data.fault.level=1;
    node_array.push_back(node_data);

    node_data.id=7;
    node_data.nodename="/cloud_calculation_front";
    node_data.fault.desc="cloud_calculation_front fault";
    node_data.rate_min = 5;
    node_data.fault.code=0x2008;
    node_data.fault.level=1;
    node_array.push_back(node_data);

    node_data.id=8;
    node_data.nodename="/cloud_calculation_back";
    node_data.fault.desc="cloud_calculation_back fault";
    node_data.rate_min = 5;
    node_data.fault.code=0x2009;
    node_data.fault.level=1;
    node_array.push_back(node_data);

    node_data.id=9;
    node_data.nodename="/cloud_calculation_left";
    node_data.fault.desc="cloud_calculation_left fault";
    node_data.rate_min = 5;
    node_data.fault.code=0x200A;
    node_data.fault.level=1;
    node_array.push_back(node_data);

    node_data.id=10;
    node_data.nodename="/cloud_calculation_right";
    node_data.fault.desc="cloud_calculation_right fault";
    node_data.rate_min = 5;
    node_data.fault.code=0x200B;
    node_data.fault.level=1;
    node_array.push_back(node_data);

    // node_data.id=11;
    // node_data.nodename="/cloud_calculation_marker";
    // node_data.fault.desc="cloud_calculation_marker fault";
    // node_data.rate_min = 10;
    // node_data.fault.code=0x200C;
    // node_data.fault.level=2;
    // node_array.push_back(node_data);

    node_data.id=12;
    node_data.nodename="/mi_stop";
    node_data.fault.desc="mi_stop fault";
    node_data.rate_min = 5;
    node_data.fault.code=0x200D;
    node_data.fault.level=1;
    node_array.push_back(node_data);
}

void Iot_comm::heart_Proc(mqtt_comm::network_heartbeat m)
{
    if(m.status == 1)
    {
        nodecheck->Find("iotheart_rate")->Beat();
    }
}

void Iot_comm::heartCallback(const mqtt_comm::network_heartbeat::ConstPtr &msg)
{
    heart_Proc(*msg); // 处理任务指令
}


// //----------3. 接收IOT消息(任务指令)-------------//
void Iot_comm::tasksCallback(const mqtt_comm::iot_tasks::ConstPtr &msg)
{
    tasks_Proc(*msg); // 处理任务指令
    Resp_tasks(*msg); // 回应任务信号
}

bool Iot_comm::refuse_task(mqtt_comm::iot_tasks t)
{
    bool res=false;
    string err_str="";

    if(t.taskType==12 && (car_state.holdcar || task_msg.cmd == "pick task")) 
    {
        err_str="pick err";
        res=true;
    }
    else if(t.taskType==13 && (!car_state.holdcar || task_msg.cmd == "release task")) 
    {
        err_str="release err";
        res=true;
    }
    // else if(task_msg.cmd!="")  res=true;

    if(res)  nh_local->setParam("refuse_task", err_str);
    
    return res;
}

void Iot_comm::tasks_Proc(mqtt_comm::iot_tasks m) // 处理任务指令
{
    if(refuse_task(m)) return;

    if (m.taskType==12)  task_msg.cmd = "pick task";
    else if (m.taskType==13)  task_msg.cmd = "release task";
    else if (m.taskType==7)  task_msg.cmd = "move task";
    else if (m.taskType==14)  task_msg.cmd = "charge task";
    else if (m.taskType==11)
    {
        nh->setParam("/show_action_enable", true);
        task_msg.cmd = "show task";
    }

    // task_msg.final_path=false;    
    task_msg.task_id = m.taskId;
    agvstate_msg.task_id = m.taskId;
    agvstate_msg.task_type = m.taskType;
    agvstate_msg.target = m.target;
    agvstate_msg.sub_target = m.sub_target;

    new_task_flag=true;

    task_tmr.Clear();
}

void Iot_comm::Resp_tasks(mqtt_comm::iot_tasks m) // 回应任务指令
{
    double image_time = ros::Time::now().toSec();
    mqtt_comm::cmd_resp resp_tasks_code;
    resp_tasks_code.header = m.header;
    resp_tasks_code.header.timestamp = to_string(image_time);
    resp_tasks_code.code = 202;
    resp_iot_tasks_pub.publish(resp_tasks_code);
}       

//----------2. 接收IOT消息(路径指令)-------------//
void Iot_comm::taskpathCallback(const mqtt_comm::iot_taskpath::ConstPtr &msg)
{
    taskpath_Proc(*msg); // 处理路径信号
    Resp_taskpath(*msg); // 回应路径信号
}

bool Iot_comm::refuse_taskpath(mqtt_comm::iot_taskpath m)
{
    string err_str="";
    bool paw_working_flag=(paw_state_str!="paw_idle" && paw_state_str!="baojia_done" && paw_state_str!="wait_for_paw_drop");

    if(new_task_flag)
    {
        new_task_flag=false;
    }
    else if(task_msg.cmd!="move task" && task_msg.cmd!="release task" && task_msg.cmd!="pick task")  
    {
        err_str="task err";
    }
    //  自转运动或自转停止4s内
    else if(car_state.turnmode==2 || selfturn_tmr.GetValue()<2 || car_ctr.turnmode==2 || selfturn_ctr>0)
    {
        err_str="self turnning";
    }
    else if(paw_working_flag)    
    {
        err_str="paw working";
    }
    else if(m.taskId!=task_msg.task_id)
    {
        err_str="task_id error";
    }
    else if(task_msg.cmd=="release task" && mi_stop_state>0)
    {
        err_str="mi_stopping";
    }

    nh_local->setParam("refuse_path", err_str);
    if(err_str!="")  
    {
        // ROS_ERROR("%s",err_str.c_str());
        return true;
    }
    else
    {
        return false;
    }
}

void Iot_comm::taskpath_Proc(mqtt_comm::iot_taskpath m) // 处理路径指令
{
    if(refuse_taskpath(m)) return;
    
    path_tmr.Clear(); 
    task_msg.stamp = ros::Time::now();
    task_msg.final_path=m.is_final_navi;
    
    if(m.dest_type == 16)  task_msg.subcmd = "0";  //shu
    else if(m.dest_type == 17)  task_msg.subcmd = "1";  //mi
    else if(m.dest_type == 18)  task_msg.subcmd = "2";  //liti

    // printf("subcmd=%s\n",task_msg.subcmd.c_str());
    //-------赋值path坐标-------//
    task_msg.path.clear();
    task_msg.path.resize(m.path.size());
    int n = m.path.size();
    // printf("n=%d\n",n);
    for (int i = 0; i < n; i++)
    {
        task_msg.path[i].pointX = m.path[i].pointX;
        task_msg.path[i].pointY = m.path[i].pointY;
        task_msg.path[i].pointHA = m.path[i].pointHA;
        task_msg.path[i].vehSpeed = m.path[i].speed_vmax;
        task_msg.path[i].lane_type = m.path[i].lane_type;
    }

    //----path路径点的坐标变换------//
    for (auto &p : task_msg.path)
        if (fabs(p.pointX) > 100000 || fabs(p.pointY) > 100000) //  come from diaodu
        {
            p.pointX -= utm_x_zero, p.pointY -= utm_y_zero;
            p.pointHA = p.pointHA / M_PI * 180;

            if (p.pointHA >= 180)  p.pointHA -= 360;
            else if (p.pointHA < -180)  p.pointHA += 360;
            // printf("%.2f %.2f %.2f speed=%.2f\n", p.pointX, p.pointY, p.pointHA, p.vehSpeed);

            p.vehSpeed/=3.6;
        }
        // (task_msg.path.end() - 1)->vehSpeed = 0.2;

    task_pub.publish(task_msg); // 给车端发路径指令

    agvstate_msg.navi_id = m.navi_id;
}

void Iot_comm::Resp_taskpath(mqtt_comm::iot_taskpath m) // 回应路径指令
{
    double image_time = ros::Time::now().toSec();
    mqtt_comm::cmd_resp resp_taskpath_code;
    resp_taskpath_code.header = m.header;
    resp_taskpath_code.header.timestamp = to_string(image_time);
    resp_taskpath_code.code = 202;
    resp_iot_taskpath_pub.publish(resp_taskpath_code);
}

void Iot_comm::MoveDirCallback(const std_msgs::String::ConstPtr &msg) // 接收控制信号
{
    move_dir=msg->data;
}

//----------1. 接收IOT消息(控制指令)-------------//
void Iot_comm::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length = msg->data;
    // printf("remain_path=%.2f\n",remain_path_length);
    // ROS_INFO("AAAAAAAAA");
}

void Iot_comm::controlsCallback(const mqtt_comm::iot_controls::ConstPtr &msg)
{
    controls_Proc(*msg); // 处理控制信号
    Resp_controls(*msg); // 回应控制信号
}


void Iot_comm::controls_Proc(mqtt_comm::iot_controls m) // 处理控制信号
{
    task_msg.stamp = ros::Time::now();
    if(m.type==2 || m.type==3)    //完成任务2
    {
        clearTask();
    }
    else if(m.type==5 && stop_flag)    //恢复行驶5
    {
        stop_flag=false;
        if(task_msg_old.cmd=="") return;
        task_msg=task_msg_old;  
        task_msg.stamp=ros::Time::now();
    }
    else if(m.type==1 ||m.type==8) //缓停
    {
        stop_flag=true;
        task_msg_old=task_msg;
        if(m.type==1)  GenStopPath(2);
        else  GenStopPath(5);
        task_msg.subcmd ="3";     //缓停
    }
    // ROS_INFO("m=%d\n",m.type); 
    task_pub.publish(task_msg); // 给车端发路径指令
}

void Iot_comm::Resp_controls(mqtt_comm::iot_controls m) // 回应控制信号
{
    double image_time = ros::Time::now().toSec();

    mqtt_comm::cmd_resp resp_ctrl_code;
    resp_ctrl_code.header = m.header;
    resp_ctrl_code.header.timestamp = to_string(image_time);
    resp_ctrl_code.code = 202;
    resp_iot_controls_pub.publish(resp_ctrl_code);
}

//----------4. 接收iot消息（充电消息）-----------//
void Iot_comm::chargeCallback(const mqtt_comm::charge_station_notify::ConstPtr &msg)
{
    charge_Proc(*msg);   
    Resp_charge(*msg);
}

void Iot_comm::charge_Proc(mqtt_comm::charge_station_notify m)
{
    // printf("charge=%d\n",m.code);
}

void Iot_comm::Resp_charge(mqtt_comm::charge_station_notify m)
{
    double image_time = ros::Time::now().toSec();
    mqtt_comm::cmd_resp resp_tasks_code;
    resp_tasks_code.header = m.header;
    resp_tasks_code.header.timestamp = to_string(image_time);
    resp_tasks_code.code = 200;
    // resp_iot_tasks_pub.publish(resp_tasks_code);
    resp_iot_charge_pub.publish(resp_tasks_code);
}

//----------1. 接收车端消息(夹爪消息)-------------//
void Iot_comm::PawStateCallback(const data_comm::paw_state::ConstPtr &msg)
{
    paw_state = *msg;
}
//----------2. 接收车端消息(速度消息)-------------//
void Iot_comm::CarStateCallback(const data_comm::car_state::ConstPtr &msg)
{
    if(car_state.turnmode==2 && msg->turnmode==0) selfturn_tmr.Clear();
    
    car_state = *msg;
    if(first_carstate<0 && car_state.ctrmode==1)
        first_carstate=1;
    else if(car_state.ctrmode==0)
        first_carstate=0;
}
//----------3. 接收车端控制消息(速度消息)-------------//
void Iot_comm::CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg)
{
    car_ctr = *msg;
}
void Iot_comm::AgvInCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (msg->data.size()==2)
    {
        agv_in_flag = msg->data[0]; // agv_in_flag
        
    }
}
void Iot_comm::LocalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    localpath = *msg;
    float localpath_passed;
    GetPathLength(localpath, 0, localpath_passed, localpath_dis);
}

void Iot_comm::ObsdisCallback(const std_msgs::Float64::ConstPtr &msg)
{
    obs_speedlimit = msg->data;
    //printf("%.2f\n",msg->data);
}

void Iot_comm::SelfTurnCallback(const std_msgs::Int32::ConstPtr &msg)
{
    selfturn_ctr=msg->data;
}

void Iot_comm::MiStopCallback(const std_msgs::Int32::ConstPtr &msg)
{
    mi_stop_state=msg->data;
}

void Iot_comm::PubIOT_AgvState() //  发布车辆状态
{
    nh->getParam("/gps_base/utmx_zero", utm_x_zero);
    nh->getParam("/gps_base/utmy_zero", utm_y_zero);
    nh->getParam("/pawcontrol/paw_state", paw_state_str);
    // printf("paw_state_str=%s\n",paw_state_str.c_str());
        
    // float task_runtime = (ros::Time::now() - task_msg.stamp).toSec(); // 任务运行时间
    if (task_msg.cmd == "pick task" && path_tmr.GetValue() > 0.8)
    {
        if (paw_state_str == "wait_for_paw_drop")  
            agvstate_msg.state_flow = 1; // 行驶中
        else if (task_msg.final_path && (paw_state_str=="paw_dropping" || paw_state_str=="paw_drop_done" || paw_state_str=="car_stop"))
            agvstate_msg.state_flow = 2; // 到达取车点
        else if (paw_state_str == "length_change_start"|| paw_state_str =="length_change_done" || paw_state_str =="baojia_start")
            agvstate_msg.state_flow = 9; // 自动取车中
        else if (paw_state_str == "baojia_done") 
            agvstate_msg.state_flow = 14; 
    }
    else if (task_msg.cmd == "release task" && path_tmr.GetValue() > 0.8)
    {
        if (paw_state_str == "baojia_done")
            agvstate_msg.state_flow = 1; // 行驶中
        else if (task_msg.final_path && remain_path_length < 0.01 && paw_state_str != "car_dropping" && paw_state_str != "wait_for_paw_drop")
            agvstate_msg.state_flow = 2; // 到达路径终点       
        else if (paw_state_str == "car_dropping")
            agvstate_msg.state_flow = 9; // 自动放车中
        // else if(agvstate_msg.state_flow ==14)   clearTask();
        else if (paw_state_str == "wait_for_paw_drop") // && path_tmr.GetValue() > 0.2)
            agvstate_msg.state_flow = 14; // 放车完成
        
        //ROS_INFO("task_msg.final_path =%d remain_path_length=%.2f  paw_state_str = %s  agvstate_msg.state_flow=%d",task_msg.final_path , remain_path_length , paw_state_str.c_str() ,agvstate_msg.state_flow);
    }
    else if (task_msg.cmd == "move task" && !stop_flag && path_tmr.GetValue() > 0.8)
    {
        static bool n_save=false;
        // if(agvstate_msg.state_flow ==2)  clearTask(); // 完成
        if (remain_path_length < 0.01  && task_msg.final_path) 
        {
            if(!n_save)
            {
                nh->setParam("/write_txt/test_saveflag", true);
                n_save=true;    
            }
            agvstate_msg.state_flow = 2;
        }
        else  
        {
            n_save=false;
            agvstate_msg.state_flow = 1; // 行驶中
        }
        // ROS_INFO("remain_path_length=%.2f  task_msg.final_path=%d agvstate_msg.state_flow=%d",remain_path_length,task_msg.final_path,agvstate_msg.state_flow);
    }
    else if(stop_flag)
    {
        if (remain_path_length < 0.01 && path_tmr.GetValue() > 0.2)  agvstate_msg.state_flow = 6;
        else  agvstate_msg.state_flow = 1; // 行驶中
    }

    agvstate_msg.task_mode = 1; // 派发模式
    if (fabs(car_state.speed[0]) < 0.001)  agvstate_msg.drv_direction = 0;
    else if(car_state.speed[0]>0)  agvstate_msg.drv_direction = 1; // agv 行驶方向 前进 后退 停止
    else agvstate_msg.drv_direction = 2;

    agvstate_msg.act_type = 3; // 车辆类型 固定为3
    agvstate_msg.in_high_voltage = agv_in_flag;

    agvstate_msg.faults = fault_array;

    resp_iot_pub.publish(agvstate_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iot_comm");
    system("sudo systemctl restart port_iot.service");

    Iot_comm iotcomm;
    ros::Rate looprate(10);
    
    while (ros::ok())
    {
        iotcomm.UpdateFaultInfo();
        iotcomm.PubIOT_AgvState();
        iotcomm.UpdateSensorFaultLevel();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
}