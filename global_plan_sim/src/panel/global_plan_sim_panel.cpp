#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QDebug>
#include <std_msgs/String.h>
#include <common/public.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <common/mydisplay.h>

#include <QDir>

#include "global_plan_sim_panel.h"

using namespace std;

namespace rviz_gui
{
    QStringList filenameInDir(string path)
    {
        //存储文件名称
        QStringList string_list;

        //判断路径是否存在
        QDir dir(QString::fromStdString(path));
        if (!dir.exists())  return string_list;

        //查看路径中后缀为.yaml格式的文件
        QStringList filters;
        filters << QString("*.yaml");
        dir.setFilter(QDir::Files | QDir::NoSymLinks); //设置类型过滤器，只为文件格式
        dir.setNameFilters(filters);                   //设置文件名称过滤器，只为filters格式

        //统计文件个数
        int dir_count = dir.count();
        // printf("%d\n", dir_count);
        // // if (dir_count <= 0)  return;

        for (int i = 0; i < dir_count; i++)
        {
            QString file_name = dir[i]; //文件名称
            file_name=file_name.left(file_name.size()-5);
            string_list.append(file_name);
        }
        return string_list;
    }

    Panel_Global_Plan_Sim::Panel_Global_Plan_Sim(QWidget *parent)
        : rviz::Panel(parent), ui(new Ui::Panel_Global_Plan_Sim)
    {
        ui->setupUi(this);

        nh = new ros::NodeHandle();
        nh_local = new ros::NodeHandle("~");

        trackpath_pub = nh->advertise<nav_msgs::Path>("/track_path", 10);
        task_pub = nh->advertise<mqtt_comm::task>("/task_cmd", 10);
        simpose_pub = nh->advertise<geometry_msgs::PoseStamped>("/sim_pose", 10);

        carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &Panel_Global_Plan_Sim::CarStateCallback, this);
        carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, &Panel_Global_Plan_Sim::CarCtrCallback, this);
        target_sub = nh->subscribe<geometry_msgs::PoseStamped>("/local_path_plan/target_pose", 10, &Panel_Global_Plan_Sim::TargetCallback, this);
        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &Panel_Global_Plan_Sim::TaskCallback, this);

        front_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/obs_dis", 10, &Panel_Global_Plan_Sim::FrontObsDisCallback, this);
        back_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_back/obs_dis", 10, &Panel_Global_Plan_Sim::BackObsDisCallback, this);
        left_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_left/obs_dis", 10, &Panel_Global_Plan_Sim::LeftObsDisCallback, this);
        right_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_right/obs_dis", 10, &Panel_Global_Plan_Sim::RightObsDisCallback, this);
        car_distance_sub = nh->subscribe<std_msgs::Float32MultiArray>("/pathtrack/car_in_out_dis", 10, &Panel_Global_Plan_Sim::CarDistanceCallback, this);

        body_front_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/agv_lidar_err", 10, &Panel_Global_Plan_Sim::BodyFrontErrCallback, this);
        body_back_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_back/agv_lidar_err", 10, &Panel_Global_Plan_Sim::BodyBackErrCallback, this);
        wheel_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/laserscan_check_angle/wheel_err", 10, &Panel_Global_Plan_Sim::WheelErrCallback, this);
    
        stop_reason_sub = nh->subscribe<std_msgs::String>("/stop_reason", 10, &Panel_Global_Plan_Sim::StopReasonCallback, this);
        move_dir_sub = nh->subscribe<std_msgs::String>("/move_dir", 10, &Panel_Global_Plan_Sim::MoveDirCallback, this);
        battery_info_sub = nh->subscribe<data_comm::battery_info>("/can_comm/battery_info", 10, &Panel_Global_Plan_Sim::BatteryInfoCallback, this);
        remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &Panel_Global_Plan_Sim::RemainPathCallback, this);

        pose_zero.header.frame_id = "base_link";
        pose_zero.pose.position.x = pose_zero.pose.position.y = pose_zero.pose.position.z = 0;
        pose_zero.pose.orientation.x = pose_zero.pose.orientation.y = pose_zero.pose.orientation.z = 0;
        pose_zero.pose.orientation.w = 1;

        qtmr.start(200);
        connect(&qtmr, SIGNAL(timeout()), this, SLOT(qtmrfunc()));

        connect(ui->btn_pick, SIGNAL(clicked()), this, SLOT(btn_pick_onclick()));
        connect(ui->btn_release, SIGNAL(clicked()), this, SLOT(btn_release_onclick()));
        connect(ui->btn_move, SIGNAL(clicked()), this, SLOT(btn_move_onclick()));
        connect(ui->btn_charge, SIGNAL(clicked()), this, SLOT(btn_charge_onclick()));
        connect(ui->btn_cleartrack, SIGNAL(clicked()), this, SLOT(btn_cleartrack_onclick()));
        connect(ui->btn_repeat, SIGNAL(clicked()), this, SLOT(btn_repeat_onclick()));

        connect(ui->btn_stop, SIGNAL(clicked()), this, SLOT(btn_stop_onclick()));
        connect(ui->btn_syscheck, SIGNAL(clicked()), this, SLOT(btn_syscheck_onclick()));
        connect(ui->btn_enable, SIGNAL(clicked()), this, SLOT(btn_enable_onclick()));
        connect(ui->btn_save, SIGNAL(clicked()), this, SLOT(btn_save_onclick()));
        connect(ui->btn_obs, SIGNAL(clicked()), this, SLOT(btn_obs_onclick()));

        char filename[1000];
        GetPackagePath("global_plan_sim", filename);
        int pos = strlen(filename);
        if (pos > 0)  filename[pos-1] = 0;
        sprintf(filename, "%s/path", filename);
        pathfilepath=filename;

        QStringList filenames = filenameInDir(pathfilepath);
        ui->cb_load->clear();
        ui->cb_load->addItems(filenames);

        string agvid, ver;
        nh->getParam("/agvId", agvid);
        nh->getParam("/version", ver);
        agvid="转运车: "+agvid+" "+ver;
        ui->label_agvid->setText(QString::fromStdString(agvid));
    }

    int Panel_Global_Plan_Sim::CheckRosNode(string name)
    {
        string msg="";
        nh->getParam(name+"/check/msg", msg);
        
        if(msg.find("OK")==msg.npos) return 1;
        else return 0;
    }

    void Panel_Global_Plan_Sim::UpdateErrCode()
    {
        string errcode="", warncode="";
        nh->getParam("/iot_comm/err", errcode);
        nh->getParam("/iot_comm/warn", warncode);
        
        char buf[100];
        sprintf(buf,"err: %s   ; warn: %s", errcode.c_str(), warncode.c_str()); 
        ui->lab8->setText(QString::fromStdString(buf));
    }

    void Panel_Global_Plan_Sim::CalculateAngle()
    {
        int n=track_path.poses.size();
        if(n<5)  return;
        
        geometry_msgs::Point p0=track_path.poses.front().pose.position;
        geometry_msgs::Point p2=track_path.poses.back().pose.position;
        geometry_msgs::Point p1=track_path.poses[int(n/2)].pose.position;
        
        float angle1=atan2(p0.y-p1.y,p0.x-p1.x)*180/M_PI;        
        float angle2=atan2(p1.y-p2.y,p1.x-p2.x)*180/M_PI;

        char buf[100];
        sprintf(buf,"car_cal: angle1=%.1f, angle2=%.1f, err=%.1f", angle1, angle2, angle1-angle2); 
        ui->lab9->setText(QString::fromStdString(buf));
    }

    void Panel_Global_Plan_Sim::PubTrackPath()  // 发布车辆走过的轨迹
    {
        track_path.header.frame_id = "map";
        track_path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose_map;
        pose_zero.header.stamp = ros::Time::now();
        transformPose("map", pose_zero, pose_map, "XXX");

        bool add_flag = true;
        if (track_path.poses.size() > 0)
        {
            geometry_msgs::PoseStamped last_pose = *(track_path.poses.end() - 1);
            float dx = last_pose.pose.position.x - pose_map.pose.position.x;
            float dy = last_pose.pose.position.y - pose_map.pose.position.y;
            float ds = sqrt(pow(dx, 2) + pow(dy, 2));
            add_flag = (ds > 0.5);
        }
        if (add_flag)  track_path.poses.push_back(pose_map);
        //  保留1000m轨迹
        if(track_path.poses.size()>1000)  track_path.poses.erase(track_path.poses.begin());

        trackpath_pub.publish(track_path);

        CalculateAngle();
    }

    void Panel_Global_Plan_Sim::qtmrfunc()  // 定时
    {
        nh_local->getParam("/gps_base/utmx_zero", utm_x_zero);
        nh_local->getParam("/gps_base/utmy_zero", utm_y_zero);

        string paw_state="", refuse_path_str="", refuse_task_str="";
        nh_local->getParam("/pawcontrol/paw_state", paw_state);
        nh_local->getParam("/iot_comm/refuse_path", refuse_path_str);
        nh_local->getParam("/iot_comm/refuse_task", refuse_task_str);
         
        int lane_type=0;
        nh->getParam("/local_path_plan/lane_type", lane_type);

        char buf[200];
        sprintf(buf, "task:%s %s  final: %d  refuse_task: %s  refuse_path: %s  lane:%d", cur_task.cmd.c_str(), cur_task.subcmd.c_str(), cur_task.final_path, refuse_task_str.c_str(), refuse_path_str.c_str(), lane_type);
        ui->lab5->setText(QString::fromUtf8(buf));

        sprintf(buf, "pawstate: %s | stop: %s  | 外纠偏: %.2f 内纠偏: %.2f", paw_state.c_str(), stop_str.c_str(), body_check_err, wheel_check_err);
        ui->lab6->setText(QString::fromUtf8(buf));

        sprintf(buf, "obs: fm%.2f fs%.2f bm%.2f bs%.2f left%.2f right%.2f | car_dis:%.2f", front_middle_obs_dis, front_side_obs_dis,back_middle_obs_dis,back_side_obs_dis,left_obs_dis,right_obs_dis, car_distance);
        ui->lab7->setText(QString::fromUtf8(buf));

        geometry_msgs::PoseStamped zero_base, zero_map;
        zero_base.header.frame_id="base_link";
        zero_base.pose.orientation.w=1;
        transformPose("map",zero_base,zero_map);
        float angle=GetYawFromPose(zero_map)*180/M_PI;
        sprintf(buf, "pose: x=%.2f y=%.2f angle=%.1f°", zero_map.pose.position.x,zero_map.pose.position.y, angle);
        ui->lab1->setText(QString::fromUtf8(buf));

        PubTrackPath();

        ui->btn_pick->setStyleSheet("background-color: rgb(186, 189, 182);");
        ui->btn_release->setStyleSheet("background-color: rgb(186, 189, 182);");
        ui->btn_move->setStyleSheet("background-color: rgb(186, 189, 182);");

        if (cur_task.cmd=="pick task")  ui->btn_pick->setStyleSheet("background-color: rgb(0, 255, 0);");
        else if (cur_task.cmd=="release task")  ui->btn_release->setStyleSheet("background-color: rgb(0, 255, 0);");
        else if (cur_task.cmd=="move task")  ui->btn_move->setStyleSheet("background-color: rgb(0, 255, 0);");

        nh_local->getParam("/pathtrack/run_enable", run_enable);
        if(run_enable)  ui->btn_enable->setStyleSheet("background-color: rgb(0, 255, 0);");
        else ui->btn_enable->setStyleSheet("background-color: rgb(186, 189, 182);");

        nh_local->getParam("/pathtrack/obs_stop_enable", obs_enable);
        if(obs_enable)  ui->btn_obs->setStyleSheet("background-color: rgb(0, 255, 0);");
        else ui->btn_obs->setStyleSheet("background-color: rgb(186, 189, 182);");

        if(repeat_flag>0)  ui->btn_repeat->setStyleSheet("background-color: rgb(0, 255, 0);");
        else ui->btn_repeat->setStyleSheet("background-color: rgb(186, 189, 182);");

        UpdateErrCode();
        // ShowActionProcess();

        // if(ui->cb_load->currentIndex()==0)  RepeatProcess1();
        RepeatProcess2();
    }

    void Panel_Global_Plan_Sim::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
    {
        cur_task = *msg;
        // ROS_INFO("final_path=%.d",cur_task.final_path);
    }

    void Panel_Global_Plan_Sim::BatteryInfoCallback(const data_comm::battery_info::ConstPtr &msg)
    {
        battery_info = *msg;  
    }

    void Panel_Global_Plan_Sim::CarDistanceCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        car_distance=msg->data[0];
    }
    
    void Panel_Global_Plan_Sim::FrontObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        front_middle_obs_dis=msg->data[0];
        front_side_obs_dis=msg->data[1];
    }

    void Panel_Global_Plan_Sim::BackObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        back_middle_obs_dis=msg->data[0];
        back_side_obs_dis=msg->data[1];
    }

    void Panel_Global_Plan_Sim::LeftObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        left_obs_dis=msg->data[0];
    }

    void Panel_Global_Plan_Sim::RightObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        right_obs_dis=msg->data[0];
    }

    void Panel_Global_Plan_Sim::BodyFrontErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if(move_dir=="front")  body_check_err=msg->data[0];
    }

    void Panel_Global_Plan_Sim::BodyBackErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if(move_dir=="back")  body_check_err=msg->data[0];
    }

    void Panel_Global_Plan_Sim::WheelErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        // if(move_dir=="front" && fabs(msg->data[1])<1)  wheel_check_err=msg->data[1];   
        // else if(move_dir=="back" && fabs(msg->data[0])<1)  wheel_check_err=msg->data[0];

        if(move_dir=="front")  wheel_check_err=msg->data[1];   
        else if(move_dir=="back")  wheel_check_err=msg->data[0];
    }

    void Panel_Global_Plan_Sim::StopReasonCallback(const std_msgs::String::ConstPtr &msg)
    {
        stop_str=msg->data;
    }

    void Panel_Global_Plan_Sim::MoveDirCallback(const std_msgs::String::ConstPtr &msg)
    {
        move_dir=msg->data;
    }

    void Panel_Global_Plan_Sim::CarStateCallback(const data_comm::car_state::ConstPtr &msg)  //接收车辆状态
    {
        char buf[200];
        sprintf(buf, "car_state:work=%d turn=%d ctr=%d vel=%.1f  battery=%d%%", msg->workmode, msg->turnmode, msg->ctrmode, msg->speed[0], battery_info.SOC);
        ui->lab3->setText(QString::fromUtf8(buf));
        car_state = *msg;
    }

    void Panel_Global_Plan_Sim::CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg)      //接收车辆状态
    {
        char buf[200];
        sprintf(buf, "car_ctr:work=%d turn=%d vel=%.1f a=%.1f", msg->workmode, msg->turnmode, msg->speed, msg->angle);
        ui->lab4->setText(QString::fromUtf8(buf));

        car_ctr=*msg;
    }

    void Panel_Global_Plan_Sim::TargetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        char buf[200];
        float angle=GetYawFromPose(*msg)*180/M_PI;
        sprintf(buf, "target:x=%.2f y=%.2f angle=%.2f", msg->pose.position.x, msg->pose.position.y, angle);
        ui->lab2->setText(QString::fromUtf8(buf));
    }

    void Panel_Global_Plan_Sim::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
    {
        remain_path_length = msg->data;
    }

    bool Panel_Global_Plan_Sim::AddPathPoint(YAML::Node node, mqtt_comm::task &task)
    {
        if(node["map_x"].IsDefined()==false)  return false;

        mqtt_comm::path_point point;
        if(node["circle"].IsDefined())
        {
            float c_x=node["map_x"].as<float>();
            float c_y=node["map_y"].as<float>();
            YAML::Node cnode=node["circle"];
            float c_r=cnode[0].as<float>();
            float c_a1=cnode[1].as<float>();
            float c_a2=cnode[2].as<float>();
            int n=cnode[3].as<int>();
            int order=cnode[4].as<int>();
            float c_da=(c_a2-c_a1)/n;
            
            point.vehSpeed = node["vel"].as<float>();
            vector<mqtt_comm::path_point> points;
            for(int i=0;i<=n;i++)
            {
                float angle=(c_a1+c_da*i)*M_PI/180;
                point.pointX = c_x+c_r*cos(angle);
                point.pointY = c_y+c_r*sin(angle);
                points.push_back(point);
            }
            
            for(int i=0;i<=n;i++)
            {
                float h_angle=0;
                if(i==n)  h_angle=180/M_PI*atan2(points[i].pointY-points[i-1].pointY, points[i].pointX-points[i-1].pointX);
                else h_angle=180/M_PI*atan2(points[i+1].pointY-points[i].pointY, points[i+1].pointX-points[i].pointX); 
                if(order==-1)  h_angle+=180;
                points[i].pointHA=h_angle;
                point.pointHA = h_angle;
                point.pointX=points[i].pointX;
                point.pointY=points[i].pointY;
                if(node["lane_type"].IsDefined())  point.lane_type=node["lane_type"].as<int>();
                else point.lane_type=0;
                task.path.push_back(point);
            }
        }
        else 
        {
            point.pointX = node["map_x"].as<float>();
            point.pointY = node["map_y"].as<float>();
            point.pointHA = node["heading"].as<float>();
            point.vehSpeed = node["vel"].as<float>();
            if(node["lane_type"].IsDefined())  point.lane_type=node["lane_type"].as<int>();
            else point.lane_type=0;
            task.path.push_back(point);
        }
        return true;
    }

    void Panel_Global_Plan_Sim::AddPathXYOff(float x_off, float y_off, float a_off)
    {
        for(auto &it:pick_task.path)
        {
            float x=it.pointX, y=it.pointY;
            it.pointX=x*cos(a_off)-y*sin(a_off);
            it.pointY=x*sin(a_off)+y*cos(a_off);
            it.pointX+=x_off,  it.pointY+=y_off;
            it.pointHA+=a_off*180/M_PI;
        }  

        for(auto &it:release_task.path)
        {
            float x=it.pointX, y=it.pointY;
            it.pointX=x*cos(a_off)-y*sin(a_off);
            it.pointY=x*sin(a_off)+y*cos(a_off);
            it.pointX+=x_off,  it.pointY+=y_off;
            it.pointHA+=a_off*180/M_PI;
        }

        for(auto &it:move_task.path)
        {
            float x=it.pointX, y=it.pointY;
            it.pointX=x*cos(a_off)-y*sin(a_off);
            it.pointY=x*sin(a_off)+y*cos(a_off);
            it.pointX+=x_off,  it.pointY+=y_off;
            it.pointHA+=a_off*180/M_PI;
        }
        
        for(auto &it:charge_task.path)
        {
            float x=it.pointX, y=it.pointY;
            it.pointX=x*cos(a_off)-y*sin(a_off);
            it.pointY=x*sin(a_off)+y*cos(a_off);
            it.pointX+=x_off,  it.pointY+=y_off;
            it.pointHA+=a_off*180/M_PI;
        }        
    }

    void Panel_Global_Plan_Sim::LoadPath(string fn)
    {
        fn=pathfilepath+"/"+fn+".yaml";
        // printf("%s\n", fn.c_str());
        // return ;

        YAML::Node config = YAML::LoadFile(fn);

        pick_task.path.clear();     pick_task.cmd="pick task";
        release_task.path.clear();  release_task.cmd="release task";
        move_task.path.clear();     move_task.cmd="move task";
        charge_task.path.clear();   charge_task.cmd="charge task"; 

        mqtt_comm::path_point point;
        for(int i=0;i<1000;i++)
        {
            char name[100];
            sprintf(name,"pose%d",i);
            if(!AddPathPoint(config["pick_path"][name], pick_task)) break;
        }
        
        release_task.subcmd=config["release_path"]["type"].as<string>();
        // ROS_INFO("%s", release_task.subcmd.c_str());

        for (int i = 0; i < 1000; i++)
        {
            char name[100];
            sprintf(name, "pose%d", i);
            if(!AddPathPoint(config["release_path"][name], release_task)) break;
        }

        for (int i = 0; i < 1000; i++)
        {
            char name[100];
            sprintf(name, "pose%d", i);
            if(!AddPathPoint(config["move_path"][name], move_task)) break;
        }

        for (int i = 0; i < 1000; i++)
        {
            char name[100];
            sprintf(name, "pose%d", i);
            if(!AddPathPoint(config["charge_path"][name], charge_task)) break;
        }

        if(config["map_x_offset"].IsDefined() && config["map_y_offset"].IsDefined() && config["map_angle_offset"].IsDefined())
        {
            float x_off=config["map_x_offset"].as<float>();
            float y_off=config["map_y_offset"].as<float>();
            float a_off=config["map_angle_offset"].as<float>();
            AddPathXYOff(x_off, y_off, a_off*M_PI/180);
        }

        if(config["next_path"].IsDefined())  next_path=config["next_path"].as<string>();
        else  next_path="";
        // ROS_INFO("%d", charge_task.path.size());
    }

    void Panel_Global_Plan_Sim::PubTask(string cmd)
    {
        btn_stop_onclick();
        usleep(50000);

        if(cmd=="pick task")  cur_task=pick_task;
        else if(cmd=="release task")  cur_task=release_task;
        else if(cmd=="move task")  cur_task=move_task;
        else if(cmd=="charge task")  cur_task=charge_task;
        else return;

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id="map";
        pose.header.stamp=cur_task.stamp;
        pose.pose.position.x=cur_task.path.front().pointX;
        pose.pose.position.y=cur_task.path.front().pointY;
        pose.pose.orientation=tf::createQuaternionMsgFromYaw(cur_task.path.front().pointHA/180.0*M_PI);
        simpose_pub.publish(pose);
        usleep(50000);
        track_path.poses.clear();

        cur_task.stamp = ros::Time::now();
        cur_task.final_path=true;
        task_pub.publish(cur_task);
    }

    void Panel_Global_Plan_Sim::btn_pick_onclick()    //  发送取车路径
    {
        string str=ui->cb_load->currentText().toStdString();
        LoadPath(str); 
        PubTask("pick task");
    }

    void Panel_Global_Plan_Sim::btn_release_onclick()    //  发送放车路径
    {
        string str=ui->cb_load->currentText().toStdString();
        LoadPath(str);
        PubTask("release task");
    }

    void Panel_Global_Plan_Sim::btn_move_onclick()    //  发送移动路径
    {
        string str=ui->cb_load->currentText().toStdString();
        LoadPath(str);
        PubTask("move task");
    }

    void Panel_Global_Plan_Sim::btn_repeat_onclick()    //  循环运动
    {
        repeat_flag=1;
        next_path="";
        nh_local->setParam("/save_date_flag", true); 
    }

    void Panel_Global_Plan_Sim::btn_charge_onclick()    //  发送充电路径
    {
        string str=ui->cb_load->currentText().toStdString();
        LoadPath(str);
        PubTask("charge task");
    }

    void Panel_Global_Plan_Sim::btn_stop_onclick()    //  发送停车指令
    {
        if(repeat_flag==0){nh_local->setParam("/save_date_flag", false);}
        show_action_flag=0;
        repeat_flag=0;
        cur_task.stamp = ros::Time::now();
        cur_task.cmd = "stop task";
        cur_task.path.clear();
        task_pub.publish(cur_task);
    }

    void Panel_Global_Plan_Sim::btn_cleartrack_onclick()    //  清除跟踪轨迹
    {
        track_path.poses.clear();
    }

    void Panel_Global_Plan_Sim::btn_syscheck_onclick()     //  系统状态检查
    {
        d_nodecheck=new dialog_node_check(this);
        d_nodecheck->setModal(true);
        d_nodecheck->show();
    }

    void Panel_Global_Plan_Sim::btn_enable_onclick()      //  系统使能
    {
        nh_local->setParam("/pathtrack/run_enable", !run_enable);
    }

    void Panel_Global_Plan_Sim::btn_obs_onclick()      //  避障使能
    {
        nh_local->setParam("/pathtrack/obs_stop_enable", !obs_enable);
    }

    // void Panel_Global_Plan_Sim::SendTurnMoveAction(mqtt_comm::path_point &show_point, float turn_angle, float move_dis, float vel)
    // {
    //     cur_task.cmd="move task";
    //     cur_task.stamp = ros::Time::now();
    //     cur_task.path.clear();    
    //     // mqtt_comm::path_point point=show_point;
    //     show_point.vehSpeed=vel;
    //     show_point.pointHA+=turn_angle;
    //     // point.pointHA=stop_angle;
    //     // point.pointX=p_map.pose.position.x;
    //     // point.pointY=p_map.pose.position.y;
    //     cur_task.path.push_back(show_point);

    //     // transformPose("map", pose_zero, p_map);
    //     // float stop_angle=GetYawFromPose(p_map)*180.0/M_PI+turn_angle;
    //     geometry_msgs::PoseStamped p_map;
    //     p_map.header.frame_id="map";
    //     p_map.pose.position.x=show_point.pointX;
    //     p_map.pose.position.y=show_point.pointY;
    //     p_map.pose.orientation=tf::createQuaternionMsgFromYaw(show_point.pointHA/180.0*M_PI);
    //     p_map=GetExtendPoseByPose(p_map, move_dis);
    //     show_point.pointX = p_map.pose.position.x;
    //     show_point.pointY = p_map.pose.position.y;
    //     cur_task.path.push_back(show_point);

    //     cur_task.final_path=true;
    //     task_pub.publish(cur_task);
    // }

    // void Panel_Global_Plan_Sim::SendReturnAction(mqtt_comm::path_point &show_point, float move_dis, float vel)
    // {
    //     // geometry_msgs::PoseStamped p_map;
    //     // transformPose("map", pose_zero, p_map);
    //     // float angle=GetYawFromPose(p_map)*180.0/M_PI;
            
    //     cur_task.cmd="move task";
    //     cur_task.stamp = ros::Time::now();
    //     cur_task.path.clear();    
    //     // mqtt_comm::path_point point;
    //     // point.vehSpeed=vel;
    //     // point.pointHA=angle;
    //     // point.pointX=p_map.pose.position.x;
    //     // point.pointY=p_map.pose.position.y;
    //     show_point.vehSpeed=vel;
    //     cur_task.path.push_back(show_point);

    //     geometry_msgs::PoseStamped p_map;
    //     p_map.header.frame_id="map";
    //     p_map.pose.position.x=show_point.pointX;
    //     p_map.pose.position.y=show_point.pointY;
    //     p_map.pose.orientation=tf::createQuaternionMsgFromYaw(show_point.pointHA/180.0*M_PI);
    //     p_map=GetExtendPoseByPose(p_map, -fabs(move_dis));
    //     show_point.pointX = p_map.pose.position.x;
    //     show_point.pointY = p_map.pose.position.y;
    //     cur_task.path.push_back(show_point);

    //     cur_task.final_path=true;
    //     task_pub.publish(cur_task);
    // }

    // void Panel_Global_Plan_Sim::SendHYAction(mqtt_comm::path_point &show_point, float move_dis, float vel)
    // {
    //     cur_task.cmd="move task";
    //     cur_task.stamp = ros::Time::now();
    //     cur_task.path.clear();    
    //     show_point.vehSpeed=vel;
    //     cur_task.path.push_back(show_point);

    //     geometry_msgs::PoseStamped p_map;
    //     p_map.header.frame_id="map";
    //     p_map.pose.position.x=show_point.pointX;
    //     p_map.pose.position.y=show_point.pointY;
    //     p_map.pose.orientation=tf::createQuaternionMsgFromYaw((show_point.pointHA+90)/180.0*M_PI);
    //     p_map=GetExtendPoseByPose(p_map, move_dis);
    //     show_point.pointX = p_map.pose.position.x;
    //     show_point.pointY = p_map.pose.position.y;
    //     cur_task.path.push_back(show_point);

    //     cur_task.final_path=true;
    //     task_pub.publish(cur_task);
    // }

    // void Panel_Global_Plan_Sim::ShowActionProcess()
    // {
    //     static mqtt_comm::path_point show_point;
    //     bool enable=false;
    //     nh_local->getParam("/show_action_enable", enable);
    //     if(enable) 
    //     {
    //         show_action_flag=1;
    //         nh_local->setParam("/show_action_enable", false);
    //     }

    //     static TTimer tmr;
    //     if(show_action_flag==1 && car_state.ctrmode==1)     //横移
    //     {
    //         geometry_msgs::PoseStamped p_map;
    //         transformPose("map", pose_zero, p_map);
    //         show_point.pointHA=GetYawFromPose(p_map)*180.0/M_PI;
    //         show_point.pointX=p_map.pose.position.x;
    //         show_point.pointY=p_map.pose.position.y;

    //         SendHYAction(show_point, 3, 0.5);
    //         show_action_flag++;   tmr.Clear();

    //         nh_local->setParam("/pathtrack/obs_stop_enable", false);
    //         nh_local->setParam("/pathtrack/selfturn_angle_err", 30);
    //     }
    //     else if(show_action_flag==2 && tmr.GetValue()>14)   //横移（改）
    //     {
    //         SendHYAction(show_point, -3, 0.5);
    //         show_action_flag++;   tmr.Clear();
    //     }
    //     else if(show_action_flag==3 && tmr.GetValue()>15)   //自转加移动
    //     {
    //         SendTurnMoveAction(show_point,45, -3, 2);
    //         show_action_flag++;   tmr.Clear(); 
    //     }
    //     else if(show_action_flag==4 && tmr.GetValue()>23)   //移动
    //     {
    //         SendTurnMoveAction(show_point, 0, 3, 2);
    //         show_action_flag++;   tmr.Clear(); 
    //     }
    //     else if(show_action_flag==5 && tmr.GetValue()>11)   //自转（改）
    //     {
    //         SendTurnMoveAction(show_point,45, 0.15, 2);
    //         show_action_flag++;   tmr.Clear(); 
    //     }
    //     else if(show_action_flag==6 && tmr.GetValue()>10)   
    //     {
    //         show_action_flag=0;   tmr.Clear();

    //         nh_local->setParam("/pathtrack/obs_stop_enable", true);
    //         nh_local->setParam("/pathtrack/selfturn_angle_err", 70);
    //     }
    //     // printf("show_action_flag=%d\n",show_action_flag);  

    // }

    void Panel_Global_Plan_Sim::RepeatProcess1()  
    {
        string paw_state="";
        nh_local->getParam("/pawcontrol/paw_state", paw_state);

        static TTimer tmr;
        if(repeat_flag==1 && tmr.GetValue()>3 && fabs(car_ctr.speed)<0.01)
        {
            ui->cb_load->setCurrentIndex(0);
            btn_pick_onclick();  
            repeat_flag=2;  tmr.Clear();
        }
        else if(repeat_flag==2 && tmr.GetValue()>3 && paw_state=="baojia_done")
        {
            btn_move_onclick();
            repeat_flag=3;  tmr.Clear();
        }
        else if(repeat_flag==3 && tmr.GetValue()>3 && fabs(car_ctr.speed)<0.01)
        {
            btn_release_onclick();
            repeat_flag=4;  tmr.Clear();

        }
        else if(repeat_flag==4 && tmr.GetValue()>3 && paw_state=="wait_for_paw_drop")
        {
            btn_move_onclick();
            repeat_flag=1;  tmr.Clear();
        }
    }

    void Panel_Global_Plan_Sim::RepeatProcess2()  
    {
        string paw_state="";
        nh_local->getParam("/pawcontrol/paw_state", paw_state);

        static TTimer tmr;
        if((repeat_flag==1 || repeat_flag==100) && tmr.GetValue()>3 && fabs(car_ctr.speed)<0.01 && remain_path_length<0.1)
        {
            if(repeat_flag==100)
            {
                if(next_path!="")  ui->cb_load->setCurrentText(QString::fromStdString(next_path));
                else  repeat_flag=0;
            }
            if(repeat_flag)
            {
                btn_pick_onclick();  
                repeat_flag=2;  tmr.Clear();
            }
        }
        else if(repeat_flag==2 && tmr.GetValue()>3 && paw_state=="baojia_done")
        {
            btn_release_onclick();
            repeat_flag=3;  tmr.Clear();
        }
        else if(repeat_flag==3 && tmr.GetValue()>3 && paw_state=="wait_for_paw_drop")
        {
            btn_move_onclick();
            repeat_flag=100;  tmr.Clear();
        }
    }

    void Panel_Global_Plan_Sim::btn_save_onclick() //  备用按钮,用于零食调试
    {
        //nh_local->setParam("/show_action_enable", true);
        // nh_local->setParam("/write_txt/test_saveflag", true);
        // system("sh /home/bit/byc4.1_ws/setup/update.sh"); 
    }

} // end namespace rviz_teleop_commander

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_gui::Panel_Global_Plan_Sim, rviz::Panel)
// END_TUTORIAL
