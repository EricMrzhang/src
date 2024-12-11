#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <common/public.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
#include <common/mydisplay.h>
#include <geometry_msgs/Twist.h>


using namespace std;
//主要实现的是纯跟踪控制
////  根据速度规划预瞄点和转向控制系数, 速度越大预瞄越远, 转角系数越小
//
class TPathTrack
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher aimpoint_pub, carctr_pub;
    ros::Subscriber localpath_sub;
    nav_msgs::Path localpath;
    
    geometry_msgs::PointStamped aimpoint;   //预瞄点坐标
    float aim_range = 2, ref_speed = 0.6;    //预瞄点距离，参考速度
    float steering_property = 1;   //转角系数
    float track_angle_err = 0;   //跟踪角度误差
    bool run_enable = false;
    
public:
    TPathTrack()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();
        //发布预瞄点
        aimpoint_pub = nh_local->advertise<geometry_msgs::PointStamped>("aimpoint", 10); // 发布预瞄点
        // 发布控制信息
        carctr_pub = nh_local->advertise<geometry_msgs::Twist>("/ctr_cmd", 10); 
        //订阅局部路径话题
        //接收局部路径 获取预瞄点
        localpath_sub = nh->subscribe<nav_msgs::Path>("/localpath", 10, &TPathTrack::LocalPathCallback, this);
        aimpoint.header.frame_id = "";
    };
    

    //函数声明
    void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    void Run();
    void UpdateCtrParam(float vel);
};

//// 接收局部路径 获取预瞄点
void TPathTrack::LocalPathCallback(const nav_msgs::Path::ConstPtr &msg) 
{
    localpath = *msg;
     //初始化
    aimpoint.point.x = aimpoint.point.y = 0;
    aimpoint.header = msg->header;
    aimpoint.header.stamp = ros::Time();   //时间戳

    if (localpath.poses.size() > 1)   //整个路径的路径点位置数
    {
        aimpoint.point = (localpath.poses.end() - 1)->pose.position;
    }
    else
    {
        aimpoint.header.frame_id = "";
        localpath.poses.clear();
        return;
    }

    float dd = 0;
    for (auto it = localpath.poses.begin(); it != localpath.poses.end()-2; ++it) //寻找预瞄点
    {
        float ds = GetDistanceXY(it->pose.position, (it + 1)->pose.position);
        dd += ds;
        if (dd >= aim_range)
        {
            aimpoint.point = it->pose.position;
            break;
        }
    }


    if (dd < aim_range)
    {
        float L1 = aim_range - dd + 0.05; //  最后引导距离
        geometry_msgs::PoseStamped p1 = *(localpath.poses.end() - 1);
        geometry_msgs::PoseStamped p2 = *(localpath.poses.end() - 2);
        p1.pose.orientation = GetQuaternionMsgByPoints(p2.pose.position, p1.pose.position);
        geometry_msgs::PoseStamped p = GetExtendPoseByPose(p1, L1);
        aimpoint.point = p.pose.position;
    }

    ref_speed = localpath.poses[0].pose.position.z; //  速度为首点Z数据
}

////  根据速度规划预瞄点和转向控制系数, 速度越大预瞄越远, 转角系数越小
void TPathTrack::UpdateCtrParam(float vel) 
{
    vel = fabs(vel);
    if (vel > 2)       
        aim_range = 4, steering_property = 2;
    else 
        aim_range = 2, steering_property = 4;
}

void TPathTrack::Run() //  主运行函数
{
     //局部路径的路径点位置数
    if (localpath.poses.size())  aimpoint_pub.publish(aimpoint);   //发布预瞄点

    geometry_msgs::Twist cmd; 
    if(localpath.poses.size()>0)  // 纯跟踪控制  
    {
        geometry_msgs::PointStamped aimpoint_base;
        transformPoint("base_link", aimpoint, aimpoint_base);
        //计算跟踪角度误差
        float track_angle_err = atan2(aimpoint_base.point.y, aimpoint_base.point.x) * 180 / M_PI;
        //根据速度，规划预瞄点和转向控制系数,
        UpdateCtrParam(ref_speed);

        cmd.linear.x=ref_speed;
        cmd.angular.z=track_angle_err*steering_property;   //角度误差*转向控制系数
    }
    else cmd.linear.x=cmd.angular.z=0;   //  无局部路径就停止

    carctr_pub.publish(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathtrack");
    TPathTrack pathtrack;   //

    ros::Rate rate(50);
    while (ros::ok())
    {
        pathtrack.Run();  ////  主运行函数
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}