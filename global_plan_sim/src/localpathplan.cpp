#include <ros/ros.h>
#include <fstream>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <common/public.h>
#include <common/mydisplay.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <mqtt_comm/task.h>
#include <mqtt_comm/path_point.h>
#include <std_msgs/Float32MultiArray.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>

using namespace std;

const float last_path_precision = 0.02;
//局部路径规划、速度规划
class TLocalPathPlan
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher taskpath_pub, globalpath_pub, localpath_pub, shotpose_pub, pathmarkers_pub, remainpath_pub, passedpath_pub;
    ros::Subscriber task_sub, lidar_target_sub, laserscan_target_sub, rviz_goal_sub, scanlidar_pos_sub, carstate_sub, carctr_sub, move_dir_sub;
    nav_msgs::Path task_path;   //  稀疏任务点
    nav_msgs::Path global_path; //  加密后的全局点(带速度规划,速度为正), 最后一段最密
    nav_msgs::Path local_path;  //  从global_path中抽取的局部路径
    geometry_msgs::PoseStamped shotpose;  //机器人相对于全局坐标系的位置和姿态
    double utm_x_zero = 0, utm_y_zero = 0;

    int Nearest_ID = 0;
    mqtt_comm::task cur_task;    //当前执行任务
    TNodeCheck *nodecheck;
    float path_remain;
    float path_passed = 0;
    string paw_state;   //夹爪状态
    data_comm::car_state cur_carstate;   //当前搬运车状态
    data_comm::car_ctr car_cmd;   //
    string move_dir="";

public:
    TLocalPathPlan()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();
        //发布的话题
        localpath_pub = nh_local->advertise<nav_msgs::Path>("localpath", 10);
        globalpath_pub = nh_local->advertise<nav_msgs::Path>("globalpath", 10);
        taskpath_pub = nh_local->advertise<nav_msgs::Path>("taskpath", 10);
        shotpose_pub = nh_local->advertise<geometry_msgs::PoseStamped>("target_pose", 10);
        pathmarkers_pub = nh_local->advertise<visualization_msgs::MarkerArray>("path_markers", 10);
        remainpath_pub = nh_local->advertise<std_msgs::Float64>("remainpath", 10);
        passedpath_pub = nh_local->advertise<std_msgs::Float64>("passedpath", 10);
        //订阅的话题
        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TLocalPathPlan::TaskCallback, this);
        carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &TLocalPathPlan::CarStateCallback, this);
        carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, &TLocalPathPlan::CarCtrCallback, this);

        lidar_target_sub = nh->subscribe<geometry_msgs::PoseStamped>("/cloud_calculation_marker/agv_pose", 10, &TLocalPathPlan::LidarPoseCallback, this);
        move_dir_sub = nh->subscribe<std_msgs::String>("/move_dir", 10, &TLocalPathPlan::MoveDirCallback, this);
        // lidar_target_sub = nh->subscribe<geometry_msgs::PoseStamped>("/cloud_calculation/agv_pose", 10, &TLocalPathPlan::LidarPoseCallback, this);
        // rviz_goal_sub = nh->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &TLocalPathPlan::RvizGoalCallback, this);
        
        //坐标系
        task_path.header.frame_id = "map";
        global_path.header.frame_id = "map";
        local_path.header.frame_id = "map";

        cur_task.cmd = "none task";
        cur_task.stamp = ros::Time::now();  //当前时间

        nodecheck = new TNodeCheck(nh_local, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(15);
    };
    //函数声明 
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void LidarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    // void RvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
    void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg);
    void MoveDirCallback(const std_msgs::String::ConstPtr &msg);

    void GetGlobalPathFromTask(mqtt_comm::task task);
    void GlobalPathPlan();
    void LocalPathPlan();
    void PathTrim();
    // nav_msgs::Path LineInterpolation(geometry_msgs::Point p1, geometry_msgs::Point p2, float ds);
    nav_msgs::Path LineInterpolation(geometry_msgs::Point p1, geometry_msgs::Point p2, float ds, float last_ds=-1);
    void SpeedPlan(nav_msgs::Path &path, float min_vel, float safe_dis = 0.3);
    bool CheckStopAtNextPoint(nav_msgs::Path path, int id);
    void Run();
 
    nav_msgs::Path GenFinalPathByPose(geometry_msgs::PoseStamped pose);
    void BrTargetLink();
    int CheckMoveMode(nav_msgs::Path path, int id);
    void PubPathMarkers();
    bool CheckTaskFinished();
    void UpdateLaneType();
};

//  发布路径速度与方向标识 task_path
////发布全局坐标系下marker列表的位置和姿态
void TLocalPathPlan::PubPathMarkers()
{
    if (task_path.poses.size() == 0)  return;

    visualization_msgs::MarkerArray markerarray;  //用于在RViz中发布多个Marker的数组
    visualization_msgs::Marker marker;
    marker.header = task_path.header; //消息的头部信息，包括frame_id和timestamp等
    
   //设置marker的属性
    marker.ns = "path_marker";
    marker.lifetime = ros::Duration(0.2);  //对象在自动删除之前应该持续多长时间。0代表永远
    marker.frame_locked = true;   //如果这个标记应该被帧锁定，即每个时间步长重新转换到它的帧
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;   //设置Marker类型
    marker.action = visualization_msgs::Marker::ADD;  //设置标记动作。选项有ADD、DELETE和new
    //设定指示线颜色
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;  //透明度，1表示完全不透明
    marker.pose.position.z = 2;   //设置模型位置
    marker.scale.z = 1;   //大小，这里表示线的粗细

    marker.id = 0;  //每个marker只能有一个id，有重复的id，只会显示一个
    //nav_msgs::Path=task_path 稀疏的任务点
    //遍历任务点
    for (int i = 0; i < task_path.poses.size(); i++)
    {
        marker.id++;
        marker.pose = GetExtendPoseByPose(task_path.poses[i], 1.5).pose;
        char text[100];
        if (i < task_path.poses.size() - 1)
            sprintf(text, "%.1f", task_path.poses[i].pose.position.z);
        else
            sprintf(text, "target");
        marker.text = text;
        markerarray.markers.push_back(marker);
    }

    marker.type = visualization_msgs::Marker::ARROW;   //设置marker类型
    marker.scale.x = 1.5;
    marker.scale.y = 0.5;
    // marker.color.r=marker.color.g=1.0f;  marker.color.b=0.0f;
    //得到相对于全局坐标系的位置和姿态
    for (int i = 0; i < task_path.poses.size() - 1; i++)
    {
        marker.id++;
        //geometry_msgs::PoseStamped机器人相对于全局坐标系的位置和姿态。
        //由一个 geometry_msgs::Pose 和一个 std_msgs::Header 组成
        geometry_msgs::PoseStamped p1 = task_path.poses[i];
        geometry_msgs::PoseStamped p2 = task_path.poses[i + 1];
        marker.pose.position.x = (p1.pose.position.x + p2.pose.position.x) * 0.5;
        marker.pose.position.y = (p1.pose.position.y + p2.pose.position.y) * 0.5;

        marker.pose.position.z = 1;
        marker.pose.orientation = GetQuaternionMsgByPoints(p1.pose.position, p2.pose.position);
        markerarray.markers.push_back(marker);
    }
    //发布全局坐标系下marker列表的位置和姿态
    pathmarkers_pub.publish(markerarray);
}

//  在rviz里模拟目标点
// void TLocalPathPlan::RvizGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     mqtt_comm::task task;
//     mqtt_comm::path_point p;
//     task.cmd = "move task";
//     task.stamp = ros::Time::now();

//     geometry_msgs::PoseStamped p0_base, p0_map;
//     p0_base.header.frame_id = "base_link";
//     p0_base.pose.orientation.w = 1;
//     transformPose("map", p0_base, p0_map);
//     p.pointHA = GetYawFromPose(p0_map) * 180 / M_PI;
//     p.pointX = p0_map.pose.position.x;
//     p.pointY = p0_map.pose.position.y;
//     p.vehSpeed = 0.4;
//     task.path.push_back(p);

//     p.pointHA = GetYawFromPose(*msg) * 180 / M_PI;
//     p.pointX = msg->pose.position.x;
//     p.pointY = msg->pose.position.y;
//     p.vehSpeed = 1;
//     task.path.push_back(p);

//     GetGlobalPathFromTask(task);
//     GlobalPathPlan();
// }

//  根据task生成全局路径 task_path 重要函数
//计算位姿、偏航角并根据其扩展路径点
void TLocalPathPlan::GetGlobalPathFromTask(mqtt_comm::task task)
{
    Nearest_ID = 0;

    task_path.header.frame_id = "map";
    task_path.header.stamp = ros::Time::now();
    task_path.poses.clear();

    shotpose.header.frame_id = "", global_path.poses.clear();

    // cout << task.path.size() << endl;

    if (task.path.empty())  return;
    //shotpose机器人相对于全局坐标系的位置和姿态
    shotpose.header.frame_id = "map";
    shotpose.header.stamp = ros::Time::now();
    shotpose.pose.position.x = task.path.back().pointX;
    shotpose.pose.position.y = task.path.back().pointY;
    shotpose.pose.position.z = task.path.back().vehSpeed;
    shotpose.pose.orientation = tf::createQuaternionMsgFromYaw(task.path.back().pointHA * M_PI / 180);
    shotpose_pub.publish(shotpose); //  发布出去
 
    //geometry_msgs::PoseStamped机器人相对于全局坐标系的位置和姿态 
    geometry_msgs::PoseStamped p_map;
    p_map.header.frame_id = "map";
    p_map.header.stamp = ros::Time::now();
    //  加入设置路径点
    for (int i = 0; i < task.path.size(); i++) 
    {
        p_map.pose.position.x = task.path[i].pointX;
        p_map.pose.position.y = task.path[i].pointY;
        p_map.pose.position.z = task.path[i].vehSpeed;

        p_map.pose.orientation = tf::createQuaternionMsgFromYaw(task.path[i].pointHA * M_PI / 180);
        task_path.poses.push_back(p_map);
    }

    // 首点距离较远，应插入首点避免存在空路径
    // geometry_msgs::PoseStamped path_first_pose=task_path.poses.front();
    // geometry_msgs::PoseStamped pose_base, pose_map;
    // pose_base.header.frame_id="base_link";
    // pose_base.header.stamp=ros::Time::now();
    // pose_base.pose.orientation.w=1;
    // transformPose("map", pose_base, pose_map);
    // geometry_msgs::PoseStamped path_first_pose_base;
    // transformPose("base_link", path_first_pose, path_first_pose_base);
    
    // float dis=GetDistanceXY(path_first_pose.pose.position,pose_map.pose.position);
    // bool add_flag=(move_dir=="front" && path_first_pose_base.pose.position.x>0); 
    // add_flag=add_flag || (move_dir=="back" && path_first_pose_base.pose.position.x<0);
    // add_flag=add_flag && (dis>0.2);
    // if(add_flag)  task_path.poses.insert(task_path.poses.begin(), pose_map);

    bool leading_enable = false;
    float leading_length = 6; // //  引导距离
    // nh_local->getParam("leading_enable",leading_enable);
    // nh_local->getParam("leading_length", leading_length);
    //GetYawFromPose:由姿态得到偏航角
    float heading = GetYawFromPose(task_path.poses.back());
   //GetAngleByPoints:根据两点计算出角度
    float line_angle = GetAngleByPoints((task_path.poses.end() - 2)->pose.position, (task_path.poses.end() - 1)->pose.position);

    bool if_go_ahead = fabs(heading - line_angle) * 180 / M_PI < 90;
    // 加入了引导点, 引导距离内千万不要放置点!!
    //task.cmd= pick task/release task 释放任务
    if (leading_enable && task.final_path && (task.cmd == "pick task" || task.cmd == "release task")) 
    {
        if (if_go_ahead)
            // 根据位姿以及偏航角扩展路径点
            p_map = GetExtendPoseByPose(task_path.poses.back(), -leading_length);  //  在最后点前插入引导点
        else 
            p_map = GetExtendPoseByPose(task_path.poses.back(), leading_length);
       
        p_map.pose.position.z = (task_path.poses.end() - 2)->pose.position.z;                                          //  引导速度

        bool add_flag = true; //  判断距离倒数第二点距离,太小就不加了，true添加
        //任务路径位姿
        if (task_path.poses.size() > 1)
        {    
            //计算倒数两个位置之间的距离
            float ds = GetDistanceXY((task_path.poses.end() - 1)->pose.position, (task_path.poses.end() - 2)->pose.position);
            if (ds < 8)
                add_flag = false; //  距离小 就不添加
        }

        if (add_flag)   //若倒数两位置的距离大时，添加点 
            task_path.poses.insert(task_path.poses.end() - 1, p_map);
    }
    //  取车路径终点延长1m,靠轮胎检测停车
    if (task.cmd == "pick task" && task.final_path) 
    {
        float L=10;
        if (!if_go_ahead) L = -L; //  判断是否倒车
        // 根据位姿以及偏航角扩展路径点
        p_map = GetExtendPoseByPose(task_path.poses.back(), L);
        *(task_path.poses.end() - 1) = p_map;
    }
}

//  接收任务指令
////计算位姿、偏航角并根据其扩展路径点
void TLocalPathPlan::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    // ROS_ERROR("%s\n", msg->cmd.c_str());
    if (msg->cmd.find("task") != string::npos)
    {
        cur_task = *msg;
        cur_task.stamp = ros::Time::now();

        GetGlobalPathFromTask(*msg);
        // printf("gggggggggggggggggggggg\n");
        GlobalPathPlan();  ////计算位姿、偏航角并根据其扩展路径点
        // printf("hhhhhhhhhhhhhhhhhhhhhh\n");
    }
    else if(msg->cmd=="")
    {
        task_path.poses.clear();
        global_path.poses.clear();
        local_path.poses.clear();
    }
}
    //搬运车状态回调函数
void TLocalPathPlan::CarStateCallback(const data_comm::car_state::ConstPtr &msg) //  接收车体状态信息
{
    cur_carstate = *msg;
}
    //搬运车控制信息回调函数
void TLocalPathPlan::CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg)
{
	car_cmd = *msg;
	// printf("%d\n", car_cmd.workmode);
}
   //移动方向回调函数
void TLocalPathPlan::MoveDirCallback(const std_msgs::String::ConstPtr &msg)
{
    move_dir=msg->data;
}

//  根据单个目标点pose生成最后路径,仅在近距离有效
//两点之间插值生成路径
nav_msgs::Path TLocalPathPlan::GenFinalPathByPose(geometry_msgs::PoseStamped pose)
{
    float Lx=8;
    if(pose.pose.position.x>=0)  Lx=-8;
    else if (pose.pose.position.x<0) Lx=8;
    // printf("pose.pose.position.x=%.2f\n",pose.pose.position.x);

    transformPose("map", pose, shotpose, "CCC");
    geometry_msgs::PoseStamped px_map;

    nav_msgs::Path path;
    ////根据偏航角添加位置点
    px_map = GetExtendPoseByPose(shotpose, Lx);

    //两点之间插值生成路径
    path = LineInterpolation(px_map.pose.position, shotpose.pose.position, 0.03);//last_path_precision);
    
    // float angle_trunk=(cur_task.path.end()-1)->pointHA;
    // for(auto &p:path.poses)  p.pose.orientation=tf::createQuaternionMsgFromYaw(angle_trunk); 
    //
    for(auto &p:path.poses)  p.pose.orientation=shotpose.pose.orientation; 
    // task_path.poses.clear();
    // task_path.poses.push_back(p1_map);
    // task_path.poses.push_back(p2_map);

    return path;
}
//geometry_msgs::PoseStamped 是 ROS (Robot Operating System) 中表示带有时间戳的姿态 (位置和方向) 的消息类型
//根据获取的雷达信息确定路径
void TLocalPathPlan::LidarPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
   //判断夹爪是否已包夹
    if(paw_state!="baojia_done")  return;
    //根据目标点进行插值得到最终路径
    global_path = GenFinalPathByPose(*msg);
    Nearest_ID = 0;
    // float angle=GetYawFromPose(shotpose)*180/M_PI;
    // printf("%.1f x=%.2f y=%.2f\n", angle, shotpose.pose.position.x, shotpose.pose.position.y);
}


//  两点之间插值生成路径
nav_msgs::Path TLocalPathPlan::LineInterpolation(geometry_msgs::Point p1, geometry_msgs::Point p2, float ds, float last_ds)
{
    //  曲线插值加密, ds为间隔距离, 最后一段特别密
    //计算两路径点为距离
    float dd = GetDistanceXY(p1, p2);

    float vel = p1.z;
    int n = dd / ds;
    if (n < 1)  n = 1;
    //
    float dx = (p2.x - p1.x) / n, dy = (p2.y - p1.y) / n;
    geometry_msgs::PoseStamped p;
    //计算角度
    float angle = atan2(p2.y - p1.y, p2.x - p1.x);
    //目标方向
    p.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
    p.header = global_path.header;
    p.pose.position = p1;
    // printf("i=%d %.2f\n", i, angle*180/M_PI);

    nav_msgs::Path path;
    path.header = global_path.header;
    //两点之间添加路径点
    for (int i = 0; i < n - 1; i++)
    {
        p.pose.position.x = p1.x + dx * i;
        p.pose.position.y = p1.y + dy * i;
        p.pose.position.z = vel; //  z 表示运动速度
        path.poses.push_back(p);
    }

    if (last_ds > 0)
    {
        n = ds / last_ds;
        if (n > 1)
        {
            p1 = p.pose.position;
            dx = (p2.x - p1.x) / n, dy = (p2.y - p1.y) / n;
            for (int i = 1; i < n; i++)
            {
                p.pose.position.x = p1.x + dx * i;
                p.pose.position.y = p1.y + dy * i;
                p.pose.position.z = vel; //  z 表示运动速度
                path.poses.push_back(p);
            }
        }
    }
    return path;
}


// 根据车头航向角与路径方向判断横移动作
int TLocalPathPlan::CheckMoveMode(nav_msgs::Path path, int id)
{
    int res = 0;

    if (path.poses.size()) //  根据预瞄点判断是否横移
    {
        geometry_msgs::PoseStamped p1 = path.poses[id];
        geometry_msgs::PoseStamped p2 = path.poses[id + 1];
        //得到偏航角
        float pose_angle = GetYawFromPose(p1);
        //到两点间与x轴的夹角,偏转角
        float path_angle = GetAngleByPoints(p1.pose.position, p2.pose.position);
        float anglex = (path_angle - pose_angle) * 180 / M_PI;

        // printf("%.2f %.2f %.2f\n", pose_angle * 180 / M_PI, path_angle * 180 / M_PI,  anglex);
        //判断是否横移
        if (anglex > 180)
            anglex -= 360;
        else if (anglex < -180)
            anglex += 360;

        if (fabs(anglex + 90) < 20)
            res = -3; //  right move   
        else if (fabs(anglex - 90) < 20)
            res = 3; // left move
    }

    return res;
}

//  检测是否停止
//包括检查是否横移动作、车头朝向是否改变、检查路径方向是否改变
bool TLocalPathPlan::CheckStopAtNextPoint(nav_msgs::Path path, int id)
{
    bool res = false;
    if (id >= path.poses.size() - 2)
        return true;

    //  检查运动模式是否改变
    //CheckMoveMode根据车头航向角与路径方向判断横移动作
    //返回值res:右移、左移,不变  
    int cur_mode = CheckMoveMode(path, id);
    int next_mode = CheckMoveMode(path, id + 1);
    if (cur_mode != next_mode)
    {
        // ROS_INFO("STOP 1 at %d",id);
        return true;
    }

    float line_varangle_max = 40; //  直线允许最大变化角度

    //  检查车头朝向是否改变
    //计算连续两个位置的偏航角
    float heading1 = GetYawFromPose(path.poses[id]) * 180 / M_PI;
    float heading2 = GetYawFromPose(path.poses[id + 1]) * 180 / M_PI;
    //偏航角的变化量的绝对值
    float dangle=fabs(heading1 - heading2);

    if(dangle>=270)  dangle-=360;
    // ROS_INFO("%d--%d heading1=%.2f heading2=%.2f dangle=%.2f",id,id+1,heading1,heading2,dangle);
    if (fabs(dangle) > line_varangle_max)
    {
        // ROS_INFO("STOP 2 at %d heading1=%.2f heading2=%.2f",id,heading1,heading2);
        return true;
    }

    //  检查路径方向是否改变
    // GetAngleByPoints两点间与x轴的夹角
    //判断连续两个位置点与x轴方向的夹角是否变化
    float line_angle1 = GetAngleByPoints(path.poses[id].pose.position, path.poses[id + 1].pose.position) * 180 / M_PI;
    float line_angle2 = GetAngleByPoints(path.poses[id + 1].pose.position, path.poses[id + 2].pose.position) * 180 / M_PI;
    dangle=fabs(line_angle1 - line_angle2);
    if(dangle>=270)  dangle-=360;
    // ROS_INFO("%d--%d line_angle1=%.2f line_angle2=%.2f dangle=%.2f",id,id+1,line_angle1,line_angle2,dangle);
    if (fabs(dangle) > line_varangle_max)
    {
        // ROS_INFO("STOP 3 at %d line_angle1=%.2f line_angle2=%.2f",id,line_angle1,line_angle2);
        return true;
    }

    return false;
}

//  速度规划
///根据上一位置速度和当前速度的变化量与距离的比值判断速度的大小，进行加速和减速
void TLocalPathPlan::SpeedPlan(nav_msgs::Path &path, float min_vel, float safe_dis)
{
    float acc_per_len=6.0/10;//, dec_per_len=-6.0/20;   //  速度/距离系数

    // 加速约束,从前往后
    // if(fabs(cur_carstate.speed[0])>0.1)  path.poses.begin()->pose.position.z=fabs(cur_carstate.speed[0]);
    // else  path.poses.begin()->pose.position.z = min_vel;
    path.poses.begin()->pose.position.z=max(min_vel, fabs(car_cmd.speed)); //最大速度
    
    for (int i = 1; i < path.poses.size(); i++)
    {   
        //速度
        float ref_vel = path.poses[i].pose.position.z;
        //当前速度
        float cur_vel = path.poses[i - 1].pose.position.z;
        //计算两点间的距离
        float ds = GetDistanceXY(path.poses[i].pose.position, path.poses[i-1].pose.position);
        //加速度
        float vk = (ref_vel - cur_vel) / ds;

        if (vk > acc_per_len)  vk = acc_per_len;   // 加速限制
        else if (vk < -acc_per_len*0.5)  vk = -acc_per_len*0.5;   // 减速限制
        path.poses[i].pose.position.z = cur_vel + vk * ds;
    }

    // 减速约束,从后往前
    float len = 0;
    (path.poses.end() - 1)->pose.position.z = min_vel;   //最小速度
    for (int i = path.poses.size() - 2; i > 0; i--)
    {    
       //根据上一位置速度和当前速度的变化量与距离的比值判断速度的大小，进行加速和减速
        float ref_vel = path.poses[i].pose.position.z;
        float cur_vel = path.poses[i + 1].pose.position.z;
        //计算两点间的距离
        float ds = GetDistanceXY(path.poses[i].pose.position, path.poses[i + 1].pose.position);
        float vk = (ref_vel - cur_vel) / ds;
        len += ds;

        if (fabs(vk) < 0.001)  break;

        if (vk > acc_per_len*0.5)  vk = acc_per_len*0.5;   //  减速限制
        else if (vk < -acc_per_len)  vk = -acc_per_len;    //  加速限制
         
        //判断是否满足安全距离
        if (len <= safe_dis)
            path.poses[i].pose.position.z = min_vel;
        else
            path.poses[i].pose.position.z = cur_vel + vk * ds;
    }

    // for(auto p : path.poses)  printf("%.2f ", p.pose.position.z);
    // printf("\n\n");
}

//  根据task_path生成全局路径golbal_path 1 路径点插值 2 加减速规划
void TLocalPathPlan::GlobalPathPlan()
{
    Nearest_ID = 0;
    global_path.header.stamp = ros::Time::now();
    if (task_path.poses.size() < 2)
    {
        global_path.poses.clear();
        return;
    }

    global_path.poses.clear();
    nav_msgs::Path plan_path; //  速度规划的路径
    // ROS_ERROR("%d %s\n",task_path.poses.size(), cur_task.cmd.c_str());
    
    for (int i = 0; i < task_path.poses.size() - 1; i++)
    {
        geometry_msgs::Point p1, p2;
        //相邻两个task_path的稀疏路径点
        p1.x = task_path.poses[i].pose.position.x;
        p1.y = task_path.poses[i].pose.position.y;
        p2.x = task_path.poses[i + 1].pose.position.x;
        p2.y = task_path.poses[i + 1].pose.position.y;
        p1.z = p2.z = task_path.poses[i].pose.position.z;
        //t
        //检测是否停止
        //包括检查是否横移动作、车头朝向是否改变、检查路径方向是否改变
        //判断搬运车是否停止
        bool stop_flag=CheckStopAtNextPoint(task_path, i);
        
        //  路径加密处理
        float ds = 0.2;
        nav_msgs::Path path;
        //LineInterpolation两点之间插值生成路径
        //task_path路径点之间进行插入路径点

        if(stop_flag)  path = LineInterpolation(p1, p2, ds, last_path_precision);
        else  path = LineInterpolation(p1, p2, ds);


        // if (i == task_path.poses.size() - 2)  ds = last_path_precision;
        
        for (auto &p : path.poses)
            p.pose.orientation = task_path.poses[i].pose.orientation;

        plan_path.header = path.header;
        for (auto p : path.poses)
            plan_path.poses.push_back(p);

        //  判断停止点
        if (stop_flag) // 有折点要停止,整体进行速度规划, 加入全局路径
        {
            float safe_dis = 0.3;
            SpeedPlan(plan_path, 0.1, safe_dis);
            for (auto p : plan_path.poses)  global_path.poses.push_back(p);
            plan_path.poses.clear();
        }
    }

    PathTrim();    //去除身后点
    if (global_path.poses.size()>0)
        (global_path.poses.end()-1)->pose.position.z = 0;

    // printf("globalpath size=%d\n", global_path.poses.size());
}
//去除身后点
void TLocalPathPlan::PathTrim()
{
    Nearest_ID=0;
    ////  获得路径上最近点ID
    FindNearestPointInPath(global_path, Nearest_ID, 2);
    nav_msgs::Path tmp_path=global_path;
    
    // 去除身后点
    global_path.poses.clear();
    for(int i=Nearest_ID;i<tmp_path.poses.size();i++)
    {
        global_path.poses.push_back(tmp_path.poses[i]);
    }  

    if(global_path.poses.size()<3)
        global_path.poses.clear();
    // 去掉短小首路径
    // tmp_path=global_path;
    // float first_len=0;
    // Nearest_ID=0;
    // for(int i=0;i<tmp_path.poses.size()-1;i++)
    //     if(CheckStopAtNextPoint(tmp_path, i))  
    //     {
    //         Nearest_ID=i+1;  break; 
    //     }
    //     else 
    //     {
    //         geometry_msgs::Point p1=tmp_path.poses[i].pose.position;
    //         geometry_msgs::Point p2=tmp_path.poses[i+1].pose.position;
    //         first_len+=GetDistanceXY(p1,p2);
    //     }
    // if(first_len<0.1 && Nearest_ID>0)
    // {
    //     global_path.poses.clear();
    //     for(int i=Nearest_ID;i<tmp_path.poses.size();i++)
    //     {
    //         global_path.poses.push_back(tmp_path.poses[i]);
    //     }
    // }
    Nearest_ID=0;
}

//判断任务是否结束
bool TLocalPathPlan::CheckTaskFinished()
{   
    //计算任务运行时间
    float task_dt = (ros::Time::now() - cur_task.stamp).toSec(); //  任务运行时间
    //夹爪状态
    nh_local->getParam("/pawcontrol/paw_state", paw_state);

    bool res = false;
    //全局路径点、path_remain 、任务运行时间、当前是否有控制指令
    if ((global_path.poses.size() == 0 || path_remain < 0.02) && task_dt > 4 && cur_task.cmd.find("task") != string::npos)
        res = true;
    if (cur_task.cmd == "pick task" && (paw_state == "baojia_start" || paw_state == "baojia_done") && task_dt > 4)
        res = true;

    return res;
}

//根据寻找任务路径上最近点更新Lanetype
void TLocalPathPlan::UpdateLaneType()
{
    // 寻找任务路径上最近点，如果加入引导点，则存在问题
    int id=0;
    //获得路径上最近点ID
    FindNearestPointInPath(task_path, id, 2);
    // printf("id=%d\n",id);
    if(id>=0 && id<cur_task.path.size())
    {
        static int lane_type=0;
        if(lane_type!=cur_task.path[id].lane_type)
        {
            lane_type=cur_task.path[id].lane_type;
            nh_local->setParam("lane_type", lane_type);
        }
    }
}

//  局部路径规划
void TLocalPathPlan::LocalPathPlan()
{
    local_path.header.stamp = ros::Time::now();
    local_path.poses.clear();

    UpdateLaneType();  //根据寻找任务路径上最近点,更新Lanetype
    //获得路径上最近点ID
    FindNearestPointInPath(global_path, Nearest_ID, 2);
    // printf("near=%d\n",Nearest_ID);

    // 获得已经走过的路程和剩余路程
    GetPathLength(global_path, Nearest_ID, path_passed, path_remain);

    std_msgs::Float64 data_msg;
    data_msg.data = path_remain;
    remainpath_pub.publish(data_msg);
    data_msg.data = path_passed;
    passedpath_pub.publish(data_msg);

    // printf("near=%d %d\n",Nearest_ID, global_path.poses.size());
    //判断任务是否结束
    if (CheckTaskFinished())  
    {
        global_path.poses.clear();
        return;
    }    
    // printf("task=%s path_remain=%.2f paw_state=%s\n", cur_task.cmd.c_str(), path_remain, paw_state.c_str());
    float ds = 0;
    for (int i = Nearest_ID; i < global_path.poses.size(); i++)
    {
        local_path.poses.push_back(global_path.poses[i]);

        // if(CheckStopAtNextPoint(global_path,i) && ds<0.1)
        //     local_path.poses.clear();
        
        if (i > 0)
        {   
            //获取相邻两点间为位置
            geometry_msgs::Point p1 = global_path.poses[i].pose.position;
            geometry_msgs::Point p2 = global_path.poses[i - 1].pose.position;
            //计算运行路程
            ds += GetDistanceXY(p1, p2);
        }

        // if (ds>20 || (CheckStopAtNextPoint(global_path, i) && ds>=0.1))、
        //判断运行路程是否超过20，且检查是是否停止，包括检查是否横移动作、车头朝向是否改变、检查路径方向是否改变
        if (ds>20 || (CheckStopAtNextPoint(global_path, i)))
        {
            if (i < global_path.poses.size() - 1)
                local_path.poses.push_back(global_path.poses[i + 1]);
            break;
        }
    }
    // printf("local_path_size=%d\n",local_path.poses.size());
}

// // 发布target_link, 必须有 shotpose
//将四元数转换为欧拉角
void TLocalPathPlan::BrTargetLink()
{
    // ROS_INFO("aaa=%s",shotpose.header.frame_id.c_str());

    // printf("111111111111111111111\n");
    //shotpose：全局坐标系下的位置和姿态
    if (shotpose.header.frame_id == "")
        return;
    //创建一个 TransformBroadcaster ，需要它来广播转换
    static tf::TransformBroadcaster br1;  
    // 
    tf::Transform transform;
    tf::Quaternion quaternion;

    tf::Vector3 origin = {shotpose.pose.position.x, shotpose.pose.position.y, 0};
    transform.setOrigin(origin);  //创建一个transform对象，2D位置信息转变为3D的transform中
    //四元数->欧拉角
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(shotpose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
     //quaternionMsgToTF可以将四元数转换为TF向量
    tf::quaternionMsgToTF(tf::createQuaternionMsgFromYaw(yaw), quaternion);
    transform.setRotation(quaternion); // base_link在map中的旋转四元数
    //发送变换
    br1.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "target_link"));
}


//  主运行函数
void TLocalPathPlan::Run()
{
    // nh_local->setParam("lane_type", 0);
    nh_local->getParam("/gps_base/utmx_zero", utm_x_zero);
    nh_local->getParam("/gps_base/utmy_zero", utm_y_zero);
    // printf("AAAAAAAAAAAAAAAAAAAAAA\n");
    static TTimer tmr;
    if (tmr.GetValue() > 1)   //计算时间
    {
        tmr.Clear();    
        // printf("BBBBBBBBBBBBBBBBBBBBBB\n");
        taskpath_pub.publish(task_path);   // task_path稀疏任务点
        // printf("CCCCCCCCCCCCCCCCCCCCCC\n");
        globalpath_pub.publish(global_path);   //global_path加密后的全局点(带速度规划,速度为正), 最后一段最密
        // printf("DDDDDDDDDDDDDDDDDDDDDD\n");
        // ROS_INFO("%d ", global_path.poses.size());
    }

    LocalPathPlan();     //局部路径规划
    // printf("EEEEEEEEEEEEEEEEEEEEEE\n");
    localpath_pub.publish(local_path);   //发布局部路径
    // printf("111111111111111111111\n");
    PubPathMarkers();  //发布全局坐标系下marker列表的位置和姿态
    // printf("FFFFFFFFFFFFFFFFFFFF\n");

    // printf("%d\n", local_path.poses.size());
    nodecheck->Find("node_rate")->Beat();
    BrTargetLink();    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "local path plan");   //节点
    TLocalPathPlan localpathplan;

    ros::Rate rate(20);
    while (ros::ok())
    {
        localpathplan.Run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
