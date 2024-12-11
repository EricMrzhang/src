#include "ros/ros.h"
#include <std_msgs/String.h>
#include "common/myudp.h"
#include <serial/serial.h>
#include <std_msgs/Float64.h>
#include <mqtt_comm/task.h>

using namespace std;

class Rfid_reader
{
private: 
    TUDP *udp;
    serial::Serial *ser;
    TNodeCheck *nodecheck;
    ros::NodeHandle *nh;
    ros::Subscriber remainpath_sub, task_sub;

    int datacount=20;
    int update_ctr=0;
    float remain_path_length=0;
    mqtt_comm::task cur_task;

public:
    unsigned char Value[100];

    Rfid_reader();
    void clear();
    void run();

    void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
};

void Rfid_reader::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    if (msg->cmd.find("task")!=string::npos)  
    {
        cur_task = *msg;
        update_ctr=0;
    }
}

void Rfid_reader::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length=msg->data;
}

Rfid_reader::Rfid_reader()
{
    nh=new ros::NodeHandle("~");
    task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &Rfid_reader::TaskCallback, this);
    remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &Rfid_reader::RemainPathCallback, this);
    
    nodecheck = new TNodeCheck(nh, "node_rate", 1.0);
    nodecheck->Find("node_rate")->SetLimit(2);
    
    ser = new serial::Serial;
    ser->setPort("/dev/ttyUSB1");
    ser->setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser->setTimeout(to);
    ser->open();

    clear();
}

void Rfid_reader::clear()
{
    for(int i=0;i<datacount*2;i++)  Value[i]=0x00;
}

void Rfid_reader::run()
{
    if(cur_task.cmd=="pick task")
    {
        float dt=(ros::Time::now()-cur_task.stamp).toSec();
        if(dt>2 && remain_path_length<5 && update_ctr==0)  update_ctr=1; 
    }
    
    unsigned char buf[20] = {0x01, 0x03, 0x02, 0x01, 0x00, 0x14, 0x15, 0xbd};
    ser->write(buf,8);
    ser->flushInput();

    usleep(60000);
    int n=ser->available();
    if(n>=5+2*datacount)
    {
        unsigned char buf[1000];
        ser->read(buf, n);
        nodecheck->Find("node_rate")->Beat();
        if(buf[0]==0x01 && buf[1]==0x03 && buf[2]==datacount*2 && buf[3]!=0x00)
        {
            memcpy(Value,buf+3, n-3);
            // for(int i=0;i<datacount*2;i++)  printf("%02x ",Value[i]);
            // printf("\n");
        }
        else clear();
    } 

    char vinstr[500]={0};   
    for(int i=0;i<12;i++)  sprintf(vinstr,"%s%02x",vinstr, Value[i]);
    nh->setParam("cargoVin", vinstr);
    // ROS_INFO(vinstr);
}

int  main(int argc, char *argv[])
{
    ros::init(argc, argv, "rfid_reader");
    Rfid_reader reader;

    ros::NodeHandle nh("~");
    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        reader.run();
        
        ros::spinOnce();
        loop_rate.sleep(); 
    }
	return 0;
}