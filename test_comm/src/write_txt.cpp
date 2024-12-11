
//包含头文件 
#include "ros/ros.h"
#include <sstream>
#include <common/can.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <common/public.h>
#include <data_comm/paw_state.h>
#include <data_comm/paw_ctr.h>
#include <data_comm/car_state.h>
#include <data_comm/car_angle.h>
#include <data_comm/car_ctr.h>
#include <test_comm/car_ctr_time.h>
#include <data_comm/battery_info.h>
#include <std_msgs/String.h>
#include <test_comm/floatmuti_time.h>
#include <test_comm/string_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <test_comm/pose_time.h>
#include <geometry_msgs/PointStamped.h>
#include <test_comm/path_msg.h>
#include <test_comm/pathpose.h>
#include <test_comm/intmuti_time.h>
#include <gps/MyGPS_msg.h>
#include <test_comm/task_time.h>
#include <mqtt_comm/task.h>
#include <test_comm/gps_time.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "mqtt_comm/resp_iot.h"

//保存数据及其对应时间

//实例化ROS句柄
ros::NodeHandle *nh;
data_comm::car_ctr car_cmd;
test_comm::car_ctr_time car_cmd_time;
data_comm::car_state car_state;
data_comm::car_angle car_angle;

test_comm::floatmuti_time wheel_err;
test_comm::floatmuti_time body_err;
test_comm::floatmuti_time wheel_distance;
test_comm::floatmuti_time obs_dis;
test_comm::floatmuti_time obs_speedlimit;
test_comm::floatmuti_time track_err;

test_comm::pose_time agv_pose, target_pose;
test_comm::pose_time aimpoint_time;
test_comm::pose_time gps_car;

test_comm::path_msg global_path;
test_comm::string_time paw_state2;

test_comm::intmuti_time agv_all_in;
test_comm::task_time task_time;
test_comm::floatmuti_time data_msg;

data_comm::car_state cur_carstate;
geometry_msgs::PoseStamped virtual_car;

mqtt_comm::resp_iot byc_state; 
mqtt_comm::task old_task,cur_task;

geometry_msgs::Point actual_pos;
geometry_msgs::Point picar_pose,hai_p1,zhu_p2;
geometry_msgs::PointStamped car_pose;
geometry_msgs::PoseStamped che_pose, gps_pose;

int flag_record = 1;
int init_time_flag=0;
string paw_state_str;
string filename;
float angle_time;
float y_dev;
float angle_dev;
TTimer save_tmr;

int n=0;
bool agv_in_flag=false, agv_allin_flag=false;
double init_time = 0;
double car_distance = 999, car_dis = 999;
float remain_path_length = 999,passed_path_length=999;
double record_time = 0;
string paw_state;
string agvId;

std::string current_time_toString()
{
    auto now = std::chrono::system_clock::now();    //记录当前时间                                                // 获取当前时间点
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000; // 毫秒数
  
    std::time_t t = std::chrono::system_clock::to_time_t(now);           // 转为 time_t 类型
    std::tm tm_now;
    localtime_r(&t, &tm_now);    // 转为本地时区的 tm 结构体
    std::stringstream ss;
    ss << std::put_time(&tm_now, "%Y-%m-%d-%H-%M-%S-"); // 格式化年月日时分秒
    ss << std::setfill('0') << std::setw(3) << ms.count(); // 格式化毫秒数，不足三位前面补零
    return ss.str();
}

void initfile(void)
{
    ofstream fs(filename);  //文件写操作
    fs<<"(time time_seconds) (cmd subcmd final_path) (scan1.x scan1.y) (scan2.x scan2.y) (scan3.x scan3.y) (scan4.x scan4.y) (target.x target.y target.angle) (actual.x actual.y actual.angle) (mi_dis) (spend_time)";
	char buf[100];
	sprintf(buf," %s\n",agvId.c_str());	
    fs<<buf;
    fs.close();  //关闭打开文件
}
//初始化变量
void initvariable()
{     
	//push_back一个用于在容器的末尾添加元素的成员函数，常用于向容器（例如向量、列表、队列等）添加新的元素
    for(int i=0;i<2;i++) wheel_err.err.push_back(0);
    body_err.err.push_back(0);
	track_err.err.push_back(0);
    for(int i=0;i<11;i++) wheel_distance.err.push_back(0);
    for(int i=0;i<7;i++) obs_dis.err.push_back(0);
    data_msg.err.push_back(0);
    for(int i=0;i<2;i++) agv_all_in.err.push_back(0);
}
//目标位姿回调函数
void TargetPoseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	target_pose.timestamp = current_time_toString();  //显示当前时间
	target_pose.pointX=msg->pose.position.x;
	target_pose.pointY=msg->pose.position.y;
	target_pose.pointHA=msg->pose.orientation.w;
}
//
void CarDistanceCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	car_distance = msg->data[0];
}

void CarDisCallback(const std_msgs::Float64::ConstPtr &msg)
{
	car_dis = msg->data;
}

//  接收车体状态信息-搬运车状态
void CarStateCallback(const data_comm::car_state::ConstPtr &msg) 
{
    cur_carstate = *msg;
}

 //  接收车体状态信息-搬运车角度
void CarAngleCallback(const data_comm::car_angle::ConstPtr &msg)
{
  	car_angle = *msg;
}

void VirtualCarCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) 
{
	virtual_car = *msg;
	//// 从ROS的四元数消息类型中得到航偏角yaw
	virtual_car.pose.position.z=tf::getYaw(msg->pose.orientation);
}

// 1.车的控制指令
void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg)
{
	car_cmd = *msg;
    car_cmd_time.timestamp = current_time_toString();
    car_cmd_time.workmode=car_cmd.workmode;
    car_cmd_time.pawmode=car_cmd.pawmode;
    car_cmd_time.turnmode=car_cmd.turnmode;
    car_cmd_time.speed=car_cmd.speed;
    car_cmd_time.angle=car_cmd.angle;
}

// 2.轮子误差
void WheelErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_err.timestamp = current_time_toString();
	wheel_err.err.clear();
    wheel_err.err.push_back(msg->data[0]);
    wheel_err.err.push_back(msg->data[1]);
}
// 3.车身误差
void BodyErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    body_err.timestamp = current_time_toString();
	body_err.err.clear();
    body_err.err.push_back(msg->data[0]);
	// printf("err_body=%.2f\n",body_err.err[0]);
}

// 4.任务路径
void processpath(mqtt_comm::task m)
{
	// if(sizeof(msg->path)<1)
	// return;
	task_time.timestamp = current_time_toString();
	task_time.cmd = m.cmd;
	task_time.subcmd = m.subcmd;
	task_time.pose.clear();
	int size_taskpath = m.path.size();
	// printf("size=%d\n",size_taskpath);
	for(int i=0;i<size_taskpath;i++)
	{
		test_comm::pathpose n;
		n.vehSpeed=m.path[i].vehSpeed;
		n.pointX=m.path[i].pointX;
		n.pointY=m.path[i].pointY;
		n.pointHA=m.path[i].pointHA;
		task_time.pose.push_back(n);
	} 
	car_pose.point.x=0;
    car_pose.point.y=0;
    car_pose.point.z=0;
}

void TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
	old_task=cur_task;
	cur_task=*msg;
	processpath(*msg);
	int flag = 0;
	if(flag==0)
	{
		// printf("%.2f",cur_task.path.end()->pointY);
		if((cur_task.path.end()-1)->pointY<-10)
		{
			// 第一个
			picar_pose.x=-91.50;
			picar_pose.y=-34.73;
			picar_pose.z=-87.7;
			hai_p1.x=-81.64;
			hai_p1.y=-20.20;
			zhu_p2.x=-91.63;
			zhu_p2.y=-19.98;
			//printf("----------------------------number 1-------------------------\n");
		}
		else
		{
			// 第二个
			picar_pose.x=-92.51;
			picar_pose.y=-1.49;
			picar_pose.z=-87.9;
			hai_p1.x=-81.80;
			hai_p1.y=5.84;
			zhu_p2.x=-92.68;
			zhu_p2.y=12.15;
			// printf("----------------------------number 2-------------------------\n");
		}
		flag++;
	}
}

// 5.跟踪误差
void TrackErrCallback(const std_msgs::Float64::ConstPtr &msg)
{
	track_err.timestamp = current_time_toString();
	track_err.err.clear();
	track_err.err.push_back(msg->data);
}

// 6. 轮子距离
void ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_distance.timestamp = current_time_toString();
	wheel_distance.err.clear();
	for(int i=0;i<11;i++) wheel_distance.err.push_back(msg->data[i]);
}

// 7.全局路径
void globalpath_pro(nav_msgs::Path m)
{
	global_path.timestamp = current_time_toString();
	// int size = sizeof(m.poses);
	int size = m.poses.size();
	test_comm::pathpose path_point;
	global_path.pose.clear();
	for(int i=0;i<size;i++)
	{
		path_point.pointHA = m.poses[i].pose.orientation.w;
		path_point.pointX = m.poses[i].pose.position.x;
		path_point.pointY = m.poses[i].pose.position.y;
		path_point.vehSpeed = m.poses[i].pose.position.z;
		global_path.pose.push_back(path_point);
	}
}

//全局路径回调函数
void GobalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
	if(msg->poses.size()<1)
	return;
	globalpath_pro(*msg); // 处理任务指令
}

// 8. 预瞄点信息
void AimpointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
	aimpoint_time.timestamp = current_time_toString();
	aimpoint_time.pointX = msg->point.x;
	aimpoint_time.pointY = msg->point.y;
	aimpoint_time.pointHA=0;
	aimpoint_time.vehSpeed=0;
}

// 9. 商品车信息
void agvPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	agv_pose.timestamp = current_time_toString();
	agv_pose.pointX=msg->pose.position.x;
	agv_pose.pointY=msg->pose.position.y;
	agv_pose.pointHA=msg->pose.orientation.w;
}
// 商品车
void carPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	che_pose=*msg;
}
// gps
void gpsPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	gps_pose=*msg;
}

//10. 前方车的距离
void ObsDistanceCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	obs_dis.timestamp = current_time_toString();
	obs_dis.err.clear();
	obs_dis.err.push_back(msg->data[0]);
	obs_dis.err.push_back(msg->data[1]);
	obs_dis.err.push_back(msg->data[2]);
}

// 11.GPS位置+偏差
void GPSDataCallback(const gps::MyGPS_msg::ConstPtr &msg)
{
	gps_car.timestamp = current_time_toString();
	gps_car.pointX = msg->map_x;
	gps_car.pointY = msg->map_y;
	gps_car.pointHA = msg->Angle;

    y_dev=car_pose.point.x-gps_car.pointX;
    angle_dev=car_pose.point.z-gps_car.pointHA;

}

// 12.速度限制
void ObsSpeedlimitCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
	obs_speedlimit.timestamp = current_time_toString();
	obs_speedlimit.err.clear();
	for(int i=0;i<6;i++)
		obs_speedlimit.err.push_back(msg->data[i]);
}

// 13. 车是不是在
void AgvInCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
	if (msg->data.size() == 2)
	{
		agv_in_flag=msg->data[0];
		agv_allin_flag=msg->data[1];
	}
}

// 14. 角速度变化率
void calculate_angle()
{
	static TTimer tmr;
	data_comm::car_ctr car_ctr_now;
	static data_comm::car_ctr car_ctr_past;
	// printf("tmr=%.2f\n",tmr.GetValue());

	if(tmr.GetValue()>0.5)
	{    
		car_ctr_now = car_cmd;
		angle_time = (car_ctr_now.angle-car_ctr_past.angle)/0.5;
		car_ctr_past = car_ctr_now;

		data_msg.err.clear();
		data_msg.timestamp = current_time_toString();
		data_msg.err.push_back(angle_time);		
		tmr.Clear();
	}
}

// 15. 时间
void time_pub()
{
	std::string currentTime = current_time_toString();
	test_comm::string_time time_msg;
	time_msg.stri=currentTime;
	time_msg.timestamp="0";
	// static TTimer tmr;
	// std_msgs::Float64 time_msg;
	// time_msg.data = tmr.GetValue();
}

//16. 夹爪状态
void paw_pub()
{
	nh->getParam("/pawcontrol/paw_state", paw_state_str);
	// std_msgs::String msg;
    paw_state2.timestamp = current_time_toString();
    paw_state2.stri = paw_state_str.c_str();    
}

//IOT
void RespIOTCallback(const mqtt_comm::resp_iot::ConstPtr &msg)
{
	byc_state = *msg;
}
//剩余路径
void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length = msg->data;
}
//走过的路径
void PassedPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    passed_path_length = msg->data;
}
//时间记录
void time_record()
{
	static TTimer tr;
	double middle_x=-85;
	static int note=1;
	
	double start_record_time;

	record_time=tr.GetValue();
	if(record_time>1){
    nh->setParam("/write_txt/time_saveflag", true);
    tr.Clear();
	};
	


// 	if(old_task.cmd=="pick task" && cur_task.cmd=="release task" && target_pose.pointX<middle_x && P2P(gps_car.pointX,gps_car.pointY,hai_p1.x,hai_p1.y)<0.3)
// 	{
// 		record_time=tr.GetValue();  //记录时间
// 		if(note) 
// 		{
// 			nh->setParam("/write_txt/time_saveflag", true);
// 			note=0;  //设置note为0，表示不再记录。
// 		}
// 	}
// 	else if(old_task.cmd=="pick task" && cur_task.cmd=="release task" && target_pose.pointX>middle_x && P2P(gps_car.pointX,gps_car.pointY,zhu_p2.x,zhu_p2.y)<0.3)
// 	{
// 		record_time=tr.GetValue();
// 		if(note) 
// 		{
// 			nh->setParam("/write_txt/time_saveflag", true);
// 			note=0;
// 		}
// 	}
// 	else if(old_task.cmd=="release task" && cur_task.cmd=="move task" && target_pose.pointX>middle_x && P2P(gps_car.pointX,gps_car.pointY,zhu_p2.x,zhu_p2.y)<0.3)
// 	{
// 		record_time=tr.GetValue();
// 		if(note) 
// 		{
// 			nh->setParam("/write_txt/time_saveflag", true);
// 			note=0;
// 		}
// 	}
// 	else if(old_task.cmd=="release task" && cur_task.cmd=="move task" && target_pose.pointX<middle_x && P2P(gps_car.pointX,gps_car.pointY,hai_p1.x,hai_p1.y)<0.3)
// 	{
// 		record_time=tr.GetValue();
// 		if(note) 
// 		{
// 			nh->setParam("/write_txt/time_saveflag", true);
// 			note=0;
// 		}
// 	}
// 	else if(init_time_flag==1 && target_pose.pointX<middle_x && P2P(gps_car.pointX,gps_car.pointY,hai_p1.x,hai_p1.y)<0.3)
// 	{
// 		record_time=999;
// 		if(note) 
// 		{
// 			nh->setParam("/write_txt/time_saveflag", true);
// 			note=0;
// 		}
// 		init_time_flag=0;
// 	}
// 	else if(init_time_flag==1 && target_pose.pointX>middle_x && P2P(gps_car.pointX,gps_car.pointY,zhu_p2.x,zhu_p2.y)<0.3)
// 	{
// 		record_time=999;
// 		if(note) 
// 		{
// 			nh->setParam("/write_txt/time_saveflag", true);
// 			note=0;
// 		}
// 		init_time_flag=0;
// 	}
// 	else note=1;

// 	if(cur_task.cmd=="pick task" && (P2P(gps_car.pointX,gps_car.pointY,hai_p1.x,hai_p1.y)<0.3 || P2P(gps_car.pointX,gps_car.pointY,zhu_p2.x,zhu_p2.y)<0.3))
// 		tr.Clear();
// 	else if(cur_task.cmd=="release task" && (P2P(gps_car.pointX,gps_car.pointY,hai_p1.x,hai_p1.y)<0.3 || P2P(gps_car.pointX,gps_car.pointY,zhu_p2.x,zhu_p2.y)<0.3))
// 		tr.Clear();
 }

//17. 保存数据
void savedata(void)
{      
    char buf[2000];

//printf("num = %d\n", num);

    sprintf(buf,
			"time = %s\n GPS_x = %.2f   GPS_y = %.2f GPS_angle = %.2f\n tar_x = %.2f   tar_y = %.2f tar_angle = %.2f\n ctr_v = %.2f   ctr_angle = %.2f\n",
			current_time_toString().c_str(), ros::Time::now().toSec() - init_time,//2   此刻时间  换成s   
            gps_car.pointX,gps_car.pointY,gps_car.pointHA,//3                           GPS x y angle
			target_pose.pointX, target_pose.pointY, target_pose.pointHA,//3             目标点 x y angle
			car_cmd_time.speed, car_cmd_time.angle);//2                                                        AGV控制量 速度 偏转角 
			//cur_carstate.speed[0],cur_carstate.speed[1],cur_carstate.speed[2],cur_carstate.speed[3], //4   AGV四轮的速度
			//car_angle.angle[4], car_angle.angle[0],car_angle.angle[1],car_angle.angle[2],car_angle.angle[3], //5 AGV实际偏转角 四轮的角度
          	//body_err.err[0], wheel_err.err[1], //2                                                            预纠偏偏差   内纠偏偏差
			// wheel_distance.err[0], wheel_distance.err[1],                                              //内雷达与车轮的x y距离
			// wheel_distance.err[3], wheel_distance.err[4],
			// wheel_distance.err[6], wheel_distance.err[7],
			// wheel_distance.err[9], wheel_distance.err[10],//8
            //obs_dis.err[0], obs_dis.err[1],//2                                                        //内前障碍物距离  外前障碍物距离	
			//car_distance, car_dis, //2                                                                 密停两车距离    实际要走的距离
			//agv_in_flag, agv_allin_flag, paw_state.c_str());//2   %.2f %d %d\n
			//);
    
    ofstream fs(filename,ios::app);
    fs<<buf;
    fs.close();
}

// void save_data_AB(void)
// {
// 	char buf[2000];
// 	float mi_dis=P2P(gps_car.pointX,gps_car.pointY,picar_pose.x,picar_pose.y);
// 	sprintf(buf,
// 			"\n %s %.2f %s %s %d %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ",
// 			current_time_toString().c_str(), ros::Time::now().toSec() - init_time,//2   此刻时间  换成s
// 			cur_task.cmd.c_str(),cur_task.subcmd.c_str(),cur_task.final_path,
// 			wheel_distance.err[0], wheel_distance.err[1],     //内雷达与车轮的x y距离
// 			wheel_distance.err[3], wheel_distance.err[4],
// 			wheel_distance.err[6], wheel_distance.err[7],
// 			wheel_distance.err[9], wheel_distance.err[10], //8
// 			target_pose.pointX,target_pose.pointY,target_pose.pointHA,
// 			gps_car.pointX,gps_car.pointY,gps_car.pointHA,
// 			mi_dis);	
// 	ofstream fs(filename,ios::app);
//     fs<<buf;
//     fs.close();		
// }
//保存时间
void save_data_time(void)
{
	char buf[2000];
	sprintf(buf," %.2f",record_time);	
	ofstream fs(filename,ios::app);
    fs<<buf;
    fs.close();		
}


ros::Subscriber carctr_sub,wheel_err_sub,body_err_sub,scanlidar_pos_sub,obs_distance_sub,pub_agv_sub,globalpath_sub,aimpoint_sub,agv_in_sub,obs_speedlimit_sub,gps_sub,task_sub,trackerr_sub,carpose_sub,gpspose_sub,carstate_sub, carangle_sub, virtualcar_sub;
ros::Subscriber target_pose_sub, car_distance_sub, car_dis_sub;
ros::Subscriber resp_iot_sub,remainpath_sub,passedpath_sub;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "write_txt");  //节点
	nh = new ros::NodeHandle("~");
    //订阅话题
	target_pose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/local_path_plan/target_pose",10, &TargetPoseCallBack);
    car_distance_sub = nh->subscribe<std_msgs::Float32MultiArray>("/pathtrack/car_in_out_dis", 10, &CarDistanceCallback);
    car_dis_sub = nh->subscribe<std_msgs::Float64>("/mi_stop/car_dis", 10, &CarDisCallback);
    //订阅话题
	carctr_sub = nh->subscribe("/pathtrack/ctr_cmd", 10, &CarCtrCallback);
    wheel_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/laserscan_check_angle/wheel_err", 10, &WheelErrCallback);
	body_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/agv_lidar_err", 10, &BodyErrCallback);
    scanlidar_pos_sub = nh->subscribe<std_msgs::Float32MultiArray>("/laserscan_check/wheel_distance", 10, &ScanPosCallback);
    obs_distance_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/obs_dis", 10, &ObsDistanceCallback);
    pub_agv_sub = nh->subscribe<geometry_msgs::PoseStamped>("/cloud_calculation/agv_pose",10, &agvPoseCallback);
	globalpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/globalpath", 10, &GobalPathCallback);
	aimpoint_sub = nh->subscribe<geometry_msgs::PointStamped>("/pathtrack/aimpoint",10,&AimpointCallback); // 发布预瞄点
    agv_in_sub = nh->subscribe<std_msgs::Int32MultiArray>("/cloud_calculation_inside/agv_in_flag", 10, &AgvInCallback);
    obs_speedlimit_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation/obs_speedlimit", 10, &ObsSpeedlimitCallback);
	gps_sub = nh->subscribe<gps::MyGPS_msg>("/gps_base/GPS_Base", 10, &GPSDataCallback);
	task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TaskCallback);
	trackerr_sub = nh->subscribe<std_msgs::Float64>("/pathtrack/track_err", 10, &TrackErrCallback);
	carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &CarStateCallback);
	carangle_sub = nh->subscribe<data_comm::car_angle>("/can_comm/car_angle", 10, &CarAngleCallback);
	virtualcar_sub = nh->subscribe<geometry_msgs::PoseStamped>("/auto_docking/virtualcar_cmd", 10, &VirtualCarCallback);

	carpose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/cloud_point_cluster/car_pose_pub",10, &carPoseCallback);
	gpspose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/cloud_point_cluster/gps_pose_pub",10, &gpsPoseCallback);

	resp_iot_sub = nh->subscribe<mqtt_comm::resp_iot>("/resp_iot", 10, &RespIOTCallback);

    remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &RemainPathCallback);
	passedpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/passedpath", 10, &PassedPathCallback);
    //保存的位置
	filename="/home/bit/record/"+current_time_toString()+".txt"; 
	// printf("%s\n", filename.c_str());

	nh->getParam("/agvId",agvId);
	initfile();
    initvariable();
	
	old_task.cmd="";

    car_pose.point.x=0;
    car_pose.point.y=0;
    car_pose.point.z=0;
    // nh->setParam("/auto_docking/dock", false);
	nh->setParam("/write_txt/test_saveflag", false);
	nh->setParam("/write_txt/time_saveflag", false);
	ros::Rate loop_rate(30);
	while (ros::ok())
	{
        static bool flag=false;
		static bool time_flag=false;
        nh->getParam("/write_txt/test_saveflag", flag);
		nh->getParam("/write_txt/time_saveflag", time_flag);
		// nh->getParam("/agvId",agvId);
        if(flag)
        {
			nh->setParam("/write_txt/test_saveflag", false);
			//save_data_AB();
			//保存数据
			savedata();
			init_time_flag=1;
            // // printf("save data begin!\n");   
			// if (init_time < 1e-7)
			// 	init_time = ros::Time::now().toSec();
            // if(save_tmr.GetValue()>0.1)       
            // {
            //     savedata();
            //     save_tmr.Clear();
            // }
        }
		if(time_flag)
		{
			nh->setParam("/write_txt/time_saveflag", false);
			//保存时间
			save_data_time();
		}

        nh->getParam("/pawcontrol/paw_state", paw_state);

		calculate_angle();   //角速度变化率
		time_pub();
		paw_pub();
		time_record();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}