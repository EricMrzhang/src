#ifndef ABC_H
#define ABC_H

//所需要包含的头文件
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h> //plugin基类的头文件
#include <std_msgs/String.h>
#include <QTimer>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#include <data_comm/battery_info.h>
#endif

#include "ui_global_plan_sim.h"
#include "node_check.h"
#include <yaml-cpp/yaml.h>

#include <mqtt_comm/task.h>
#include <mqtt_comm/path_point.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>

using namespace std;

namespace rviz_gui
{
    // 所有的plugin都必须是rviz::Panel的子类
    class Panel_Global_Plan_Sim : public rviz::Panel
    {
        // 后边需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏
        Q_OBJECT
    public:
        // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
        Panel_Global_Plan_Sim(QWidget *parent = 0);

        // 公共槽.
    public Q_SLOTS:

        // 内部槽.
    protected Q_SLOTS:
        void qtmrfunc();

        void btn_release_onclick();
        void btn_pick_onclick();
        void btn_move_onclick();
        void btn_cleartrack_onclick();
        void btn_stop_onclick();
        void btn_syscheck_onclick();
        void btn_enable_onclick();
        void btn_save_onclick();
        void btn_obs_onclick();
        void btn_charge_onclick();
        void btn_repeat_onclick();

        // 内部变量.
    protected:
        ros::NodeHandle *nh, *nh_local;
        ros::Publisher task_pub, trackpath_pub, simpose_pub;
        ros::Subscriber carstate_sub, carctr_sub, target_sub, task_sub;

        ros::Subscriber front_obs_dis_sub,back_obs_dis_sub,left_obs_dis_sub,right_obs_dis_sub, car_distance_sub, remainpath_sub;
        ros::Subscriber body_front_err_sub, body_back_err_sub, wheel_err_sub, stop_reason_sub, move_dir_sub, battery_info_sub; 

        geometry_msgs::PoseStamped pick_pose, release_pose, target_pose;
        double utm_x_zero = 0, utm_y_zero = 0;
        geometry_msgs::PoseStamped pose_zero;
        nav_msgs::Path track_path;
        data_comm::car_ctr car_ctr;
        data_comm::car_state car_state;
        data_comm::battery_info battery_info;
        bool run_enable = false, obs_enable = false;
        string pathfilepath;
        int repeat_flag=0;
        
        mqtt_comm::task pick_task, release_task, move_task, charge_task, cur_task;
        
        float front_middle_obs_dis=999, front_side_obs_dis=999;
        float back_middle_obs_dis=999, back_side_obs_dis=999, car_distance=999;
        float left_obs_dis=999,right_obs_dis=999;
        float wheel_check_err=999, body_check_err=999; 
        string move_dir, stop_str;
        float remain_path_length=0;
        string next_path="";

        bool AddPathPoint(YAML::Node node, mqtt_comm::task &task);
        void LoadPath(string fn);
        void AddPathXYOff(float x_off, float y_off, float a_off);

        void Path2Task(string cmd, nav_msgs::Path path);
        void GetPickReleasePose();
        void DispAgvState();
        void PubTrackPath();
        void PubTask(string cmd);
        void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
        void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg);
        void TargetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void FrontObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void BackObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void LeftObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void RightObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
        void CarDistanceCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

        void WheelErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void BodyFrontErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void BodyBackErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        void StopReasonCallback(const std_msgs::String::ConstPtr &msg);
        void MoveDirCallback(const std_msgs::String::ConstPtr &msg);
        void BatteryInfoCallback(const data_comm::battery_info::ConstPtr &msg);

        void TaskCallback(const mqtt_comm::task::ConstPtr &msg);

        int CheckRosNode(string name);
        void UpdateErrCode();

    private:
        Ui::Panel_Global_Plan_Sim *ui;
        dialog_node_check *d_nodecheck;
        QTimer qtmr;
        // bool matching_loc_enable;

        int show_action_flag = 0;
        // void SendTurnMoveAction(mqtt_comm::path_point &show_point, float turn_angle, float move_dis, float vel);
        // void SendReturnAction(mqtt_comm::path_point &show_point, float move_dis, float vel);
        // void SendHYAction(mqtt_comm::path_point &show_point, float move_dis, float vel);
        // void ShowActionProcess();
        void RepeatProcess1();
        void RepeatProcess2();
        void CalculateAngle();
    };
}

#endif
