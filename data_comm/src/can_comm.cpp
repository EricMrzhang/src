//添加twice轨迹跟踪
/*
		SetMode(car_state2);
		SetRunPara(car_orrentation_state2,car_speed2,car_angle2);
		SetPawMode(paw_distance_control2, paw_lift_control2, vehicle_put2, paw_distance2);
输入：
pawctr:

int32 paw_distance_control: pawdistance_mode=0x00 无动作 pawdistance_mode=0x01 轴距增大 pawdistance_mode=0x02 轴距减小
float32 paw_distance:[0,1000]
int32 paw_lift_control: pawlifting_mode=0x00 无动作  pawlifting_mode=0x01 夹爪上升  pawlifting_mode=0x02 夹爪降低
int32 vehicle_put: Vehicleput_mode=0x00 无动作 Vehicleput_mode=0xAA 自动取车 Vehicleput_mode=0xBB 自动放车

zcarctr:

int32 car_state: cmd=0x0; 初始化状态 cmd=0x1; 急停 cmd=0x2; 待机 cmd=0x3; 工作 cmd=0x4; 充电 cmd=0x5; 自检
int32 car_orrentation_state mode=0x0; 阿克曼前后轮模式 mode=0x1; 差速模式 mode=0x2; 自转模式 mode=0x3; 横移模式
float32 car_speed
float32 car_angle
// 1）（阿克曼模式）单位0.01m/s，范围[-8.5,8.5]，指令 -850~850
// 2）（横移模式）单位0.01m/s，范围[-5,5]，指令 -500~500
// 3）（自传模式）角速度值单位rad/s
// 4）（差速模式）单位0.01m/s，范围[-5,5]，指令 -500~500
// p2
// 1）（阿克曼模式）单位0.01°，范围[-100°,100°]，左为负，指令-10000~10000
// 2）（横移模式）单位0.01°，范围[-100°,100°]，左为负，指令-10000~10000
// 3）（差速模式）单位0.01m/s，范围[-5,5]，指令 -500~500
*/
#include "ros/ros.h"
#include <sstream>
#include <common/can.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <common/public.h>
#include <data_comm/paw_state.h>
#include <data_comm/paw_ctr.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#include <data_comm/battery_info.h>
#include <data_comm/car_angle.h>

//主要作用是发布状态信息
//	SetWorkMode(car_cmd);   ////将夹爪和搬运车收到的控制指令控发送
//SetRunPara(car_cmd);    //发送运行参数，包括转向模式、速度，角度，充电口开/关
//SetPawMode(paw_ctr);   //发送夹爪控制模式状态，包括左右夹爪左右距离、速度，上下，取放状态
//SetObsDis();     //发送周围障碍物的距离
//ProcCanMsg();  //根据标识符can_id，发布对应的信息,包括：搬运车状态、夹爪状态、电池信息、故障码、工作距离/时间，转角

TCan *can;
TNodeCheck *nodecheck;
//转运车状态
data_comm::car_state car_state;
//转运车转向角
data_comm::car_angle car_angle;
//转运车控制
data_comm::car_ctr car_cmd;
//夹爪状态 
data_comm::paw_state paw_state;
//夹爪控制
data_comm::paw_ctr paw_ctr;   

ros::NodeHandle *nh;
ros::Publisher carstate_pub, pawstate_pub, battery_pub,carangle_pub;

float front_middle_obs_dis=999, front_side_obs_dis=999;
float back_middle_obs_dis=999, back_side_obs_dis=999;
float left_obs_dis=999,right_obs_dis=999;

//爪子的回调函数
void PawCtrCallback(const data_comm::paw_ctr::ConstPtr &msg)
{
	paw_ctr = *msg;
}

//转运车控制的回调函数
void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg)
{
	bool moving = false;
	//判断转运车是否正在移动
	for (int i = 0; i < 4; i++)
		if (fabs(car_state.speed[i]) > 0.01)
			moving = true;

	if (car_cmd.turnmode != msg->turnmode && moving)  return;

	car_cmd = *msg;
	// printf("%d\n", car_cmd.workmode);
}

//前方障碍物距离的回调函数
void FrontObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    front_middle_obs_dis=msg->data[0];
    front_side_obs_dis=msg->data[1];
}
////后方障碍物距离的回调函数
void BackObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    back_middle_obs_dis=msg->data[0];
    back_side_obs_dis=msg->data[1];
}
//左方障碍物距离的回调函数
void LeftObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    left_obs_dis=msg->data[0];
}
//右方障碍物距离的回调函数
void RightObsDisCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    right_obs_dis=msg->data[0];
}

//设置障碍物距离的上传格式
void SetObsDis()
{  
	can->send_frame.can_id = 0x8C150A1C; 
	can->send_frame.can_dlc = 0x08;
	can->send_frame.data[0] = front_middle_obs_dis*100;
	can->send_frame.data[1] = int(front_middle_obs_dis*100)>>8;  //右移8位
	can->send_frame.data[2] = front_side_obs_dis*100;
	can->send_frame.data[3] = int(front_side_obs_dis*100)>>8;

	can->send_frame.data[4] = back_middle_obs_dis*100;
	can->send_frame.data[5] = int(back_middle_obs_dis*100)>>8;
	can->send_frame.data[6] = back_side_obs_dis*100;
	can->send_frame.data[7] = int(back_side_obs_dis*100)>>8;
	can->Send();

	can->send_frame.can_id = 0x8C140A1C; 
	can->send_frame.can_dlc = 0x08;
	can->send_frame.data[0] = left_obs_dis*100;
	can->send_frame.data[1] = int(left_obs_dis*100)>>8;
	can->send_frame.data[2] = can->send_frame.data[0];
	can->send_frame.data[3] = can->send_frame.data[1];
	can->send_frame.data[4] = right_obs_dis*100;
	can->send_frame.data[5] = int(right_obs_dis*100)>>8;
	can->send_frame.data[6] = can->send_frame.data[4];
	can->send_frame.data[7] = can->send_frame.data[5];
	can->Send();
}


// 0-初始化状态  1-急停  2-待机  3-工作  4-充电  5-自检  cmd_car   //搬运车收到的控制指令
// 0-爪子关闭   1-开启     cmd_paw    夹爪收到的控制指令

//将夹爪和搬运车收到的控制指令控发送
void SetWorkMode(data_comm::car_ctr cmd) // int cmd_car, int cmd_paw1)
{
	if(car_state.workmode==1)  cmd.workmode=2;   //若转运车状态模式为急停，则控制指令为工作模式
	
	can->send_frame.can_id = 0x8C110A1C; // | (1<<32);

	can->send_frame.can_dlc = 0x08;
	//pow(2, 3)=2**3=8
	//将夹爪模式和搬运车控制的工作状态发送

	can->send_frame.data[0] = cmd.pawmode * pow(2, 3) + cmd.workmode;

	for (int i = 1; i < 8; i++)
		can->send_frame.data[i] = 0x0;
	can->Send();
	// ROS_ERROR("%d\n", cmd.workmode);
}

// 0-阿克曼前后轮模式  1-差速模式  2-自转模式  3-横移模式
// 1）（阿克曼模式）单位0.01m/s，范围[-8.5,8.5]，指令 -850~850
// 2）（横移模式）单位0.01m/s，范围[-5,5]，指令 -500~500
// 3）（自传模式）角速度值单位rad/s
// 4）（差速模式）单位0.01m/s，范围[-5,5]，指令 -500~500
// p2
// 1）（阿克曼模式）单位0.01°，范围[-100°,100°]，左为负，指令-10000~10000
// 2）（横移模式）单位0.01°，范围[-100°,100°]，左为负，指令-10000~10000
// 3）（差速模式）单位0.01m/s，范围[-5,5]，指令 -500~500

//设置运行参数
//格式【转向模式，速度低8位，速度高8位，角度低8位，角度高8位，充电口打开，充电口关闭】
void SetRunPara(data_comm::car_ctr cmd)
{
	can->send_frame.can_id = 0x8C120A1C; // | (1<<32);

	can->send_frame.can_dlc = 0x08;

	for (int i = 0; i < 8; i++)
		can->send_frame.data[i] = 0x0;

	can->send_frame.data[0] = cmd.turnmode;    //转向模式
	// printf("%d\n",turnmode);
	int x = cmd.speed * 100;   //把速度放大100倍

	if (x < 0)    //速度为负时
		x = x + 0x10000;   
	can->send_frame.data[1] = x % 256;     //2**8    取低8位
	can->send_frame.data[2] = x / 256;    //取高8位

	x = cmd.angle * 100;    //角度
	if (x < 0)  x = x + 0x10000;
	can->send_frame.data[3] = x % 256;   //取低8位
	can->send_frame.data[4] = x / 256;   //取高8位

	if(cmd.workmode==4) can->send_frame.data[5]=1;   //充电口打开
	else can->send_frame.data[5]=0;     //充电口关闭

	static int count = 0;
	can->send_frame.data[7] = count++;

	can->Send();
}

// pawdistance_mode
// pawdistance_mode=0x00 无动作
// pawdistance_mode=0x01 轴距增大
// pawdistance_mode=0x02 轴距减小
// pawlifting_mode
// pawlifting_mode=0x00 无动作
// pawlifting_mode=0x01 夹爪上升
// pawlifting_mode=0x02 夹爪降低

// Vehicleput_mode
// Vehicleput_mode=0x00 无动作
// Vehicleput_mode=0xAA 自动取车
// Vehicleput_mode=0xBB 自动放车
// p1夹抓轴距调整值 【0，1000】

//发送夹爪控制模式状态
//包括左右夹爪左右距离、速度，上下，取放状态
void SetPawMode(data_comm::paw_ctr cmd)
{
	can->send_frame.can_id = 0x8C130A1C;
	can->send_frame.can_dlc = 0x08;

	for (int i = 0; i < 8; i++)
		can->send_frame.data[i] = 0x0;   //清零

   //初始化
	int left_pawdistance_mode_big, left_pawdistance_mode_little, right_pawdistance_mode_big, right_pawdistance_mode_little;
	left_pawdistance_mode_big = left_pawdistance_mode_little = right_pawdistance_mode_big = right_pawdistance_mode_little = 0;
     
	 //左抓距离控制
	if (cmd.left_paw_distance_control == 1)
	{
		left_pawdistance_mode_big = 1;
		left_pawdistance_mode_little = 0;
	}
	else if (cmd.left_paw_distance_control == 2)
	{
		left_pawdistance_mode_big = 0;
		left_pawdistance_mode_little = 1;
	}

     //右抓距离控制
	if (cmd.right_paw_distance_control == 1)
	{
		right_pawdistance_mode_big = 1;
		right_pawdistance_mode_little = 0;
	}
	else if (cmd.right_paw_distance_control == 2)
	{
		right_pawdistance_mode_big = 0;
		right_pawdistance_mode_little = 1;
	}
	else
	{
		right_pawdistance_mode_big = 0;
		right_pawdistance_mode_little = 0;
	}
    
	//保存左右夹爪控制状态
	can->send_frame.data[0] = right_pawdistance_mode_little * pow(2, 3) + right_pawdistance_mode_big * pow(2, 2) + left_pawdistance_mode_little * pow(2, 1) + left_pawdistance_mode_big;

    //左爪速度
	int x = cmd.left_paw_speed; //  left_wheel_paw * 1;
	if (x < 0)
		x = x + 0x10000;
	can->send_frame.data[1] = x % 256;
	can->send_frame.data[2] = x / 256;
	//右抓速度
	x = cmd.right_paw_speed;
	if (x < 0)
		x = x + 0x10000;
	can->send_frame.data[5] = x % 256;
	can->send_frame.data[6] = x / 256;

	static int paw_cmd=0;
	//夹爪上下状态控制
	can->send_frame.data[3] = cmd.paw_lift_control;
	//车辆取放状态控制
	can->send_frame.data[4] = cmd.vehicle_put_control;
	// if((paw_cmd==0xAA && cmd.vehicle_put_control==0xBB) || (paw_cmd==0xBB && cmd.vehicle_put_control==0xAA))
	// {
	// 	ROS_INFO("PAW_ctr err!! \n");
	// }cur_carstate.turnmode
	paw_cmd=cmd.vehicle_put_control;

	static int count = 0;
	can->send_frame.data[7] = count++;

	can->Send();
}

//根据标识符can_id，发布对应的信息
void ProcCanMsg()
{
	for (int i = 0; i < can->rec_frames.size(); i++)
	{
		can_frame *frame = &can->rec_frames[i];
		// printf("frame=%x\n",frame->can_id);

        //// AGV状态信息
		if (frame->can_id == 0x88411B0A) // AGV状态信息
		{
			nodecheck->Find("can_rate")->Beat();
             //搬运车状态：工作模式、是否转向、控制模式、holdcar
			car_state.workmode = (frame->data[0] & 0x7);
			car_state.turnmode = (frame->data[0] >> 3 & 0x3);
			car_state.ctrmode = !(frame->data[0] >> 5 & 0x1);
			car_state.holdcar = (frame->data[0] >> 6 & 0x1); 
			// printf("workmode=%d\n",car_state.workmode);

			for (int i = 0; i < 4; i++)
				car_state.speed[i] = char(frame->data[1 + i]) * 0.1;
			//发布搬运车状态
			carstate_pub.publish(car_state);
			// printf("%02x \n",car_state.turnmode);

			// int id = frame->data[5] * 256 + frame->data[6];
 			// int charge = frame->data[6];   //充电
		}
		//夹爪状态
		else if (frame->can_id == 0x88421C0A) //  Paw state
		{
			nodecheck->Find("can_rate")->Beat();
			// printf("six_bite=%d\n",frame->data[6]);

			paw_state.updown_state = frame->data[0];
			paw_state.left_axischange_state = frame->data[1];
			paw_state.carhold_state = frame->data[2];               //0x00无动作 0x01自动取车中 0x02自动取车完成0x03自动放车中 0x04自动放车完成
			paw_state.right_axischange_state = frame->data[3];
			paw_state.paw_state=frame->data[5];                     //liuzhi_1109::0收回 1张开 2动作中
			
			// ROS_INFO("carhold_state=%d",paw_state.carhold_state);
			pawstate_pub.publish(paw_state);
		}
		//电池状态
		else if (frame->can_id == 0x88431C0A) //  电池状态
		{
			nodecheck->Find("can_rate")->Beat();

			data_comm::battery_info msg;
			msg.SOC = frame->data[0];
			msg.current = (frame->data[1] * 256 + frame->data[2]) * 0.1;
			msg.voltage = (frame->data[3] * 256 + frame->data[4]) * 0.1;
			msg.SOH = frame->data[5];
			msg.DCDC = frame->data[6];
			battery_pub.publish(msg);
			// printf("AAA=%d\n", msg.SOC);
		}
		//  故障码
		else if (frame->can_id == 0x88441C0A)  
		{
            nodecheck->Find("can_rate")->Beat();

			car_state.errcode=frame->data[0]*256*256*256;
			car_state.errcode+=frame->data[1]*256*256;
			car_state.errcode+=frame->data[2]*256;
			car_state.errcode+=frame->data[3];
		}
		//工作里程、时间
		else if (frame->can_id == 0x88451C0A)
		{
            nodecheck->Find("can_rate")->Beat();

			car_state.odom = (frame->data[0] + frame->data[1] * 256)*0.1;		   // 工作里程  单位：0.1公里
			car_state.runningtime = (frame->data[3] + frame->data[4] * 256)*0.1;   // 工作时间  单位：小时
			// ROS_INFO("%.1f %.1f", car_state.odom, car_state.runningtime);    
			// paw_state.paw_state=frame->data[5];
		}
		 //车轮转角
		else if (frame->can_id ==0x88461C0A) //车轮转角
		{
			nodecheck->Find("can_rate")->Beat();
			if(frame->data[1]>127) car_angle.angle[0] = -((256-frame->data[0]) + (255-frame->data[1]) * 256)*0.1;
			else car_angle.angle[0] = (frame->data[0] + frame->data[1] * 256)*0.1; 
			if(frame->data[3]>127) car_angle.angle[1] = -((256-frame->data[2]) + (255-frame->data[3]) * 256)*0.1;
			else car_angle.angle[1] = (frame->data[2] + frame->data[3] * 256)*0.1; 
			if(frame->data[5]>127) car_angle.angle[2] = -((256-frame->data[4]) + (255-frame->data[5]) * 256)*0.1;
			else car_angle.angle[2] = (frame->data[4] + frame->data[5] * 256)*0.1; 
			if(frame->data[7]>127) car_angle.angle[3] = -((256-frame->data[6]) + (255-frame->data[7]) * 256)*0.1;
			else car_angle.angle[3] = (frame->data[6] + frame->data[7] * 256)*0.1; 
			
			float x0=5438.0/2;
			float s1=x0/(tan(car_angle.angle[0]*M_PI/180));
			float s2=x0/(tan(car_angle.angle[2]*M_PI/180));
			float s3=2*x0/(s1+s2);
			car_angle.angle[4] = 180/M_PI*atan(s3);
			// ROS_INFO("angle=%.2f %.2f %.2f %.2f %.2f",car_angle.angle[0],car_angle.angle[1],car_angle.angle[2],car_angle.angle[3],car_angle.angle[4]);
			carangle_pub.publish(car_angle);
		}

		frame->can_id = 0;
	}
}

//can0通信重启，显示信息
void can0_restart()
{
    system("sudo ifconfig can0 down");
    system("sudo tc qdisc del dev can0 root");
    system("sudo ip link set can0 type can restart-ms 100");
    system("sudo ip link set can0 type can bitrate 250000 sample-point 0.75");
    system("sudo ip link set can0 type can berr-reporting on");
    system("sudo ifconfig can0 txqueuelen 1000");
    system("sudo tc qdisc add dev can0 root handle 1: pfifo");
    system("sudo ifconfig can0 up");
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "plus1");   //节点名称

	nh = new ros::NodeHandle("~");
	nodecheck = new TNodeCheck(nh, "node_rate can_rate");
	nodecheck->Find("node_rate")->SetLimit(10);

	int can_ch = 0;
	can = InitCan(can_ch, 500000);

    // 订阅话题
	ros::Subscriber pawctr_sub = nh->subscribe("/pawcontrol/paw_ctr", 10, &PawCtrCallback);
	ros::Subscriber carctr_sub = nh->subscribe("/pathtrack/ctr_cmd", 1, &CarCtrCallback);

    //advertise()括号里面的第一个参数是话题名字，第二个参数是用于发布消息的消息队列的大小。
	//发布话题
	carstate_pub = nh->advertise<data_comm::car_state>("car_state", 10);
	carangle_pub = nh->advertise<data_comm::car_angle>("car_angle", 10);
	pawstate_pub = nh->advertise<data_comm::paw_state>("paw_state", 10);
	battery_pub = nh->advertise<data_comm::battery_info>("battery_info", 10);
     //订阅话题
	ros::Subscriber front_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_front/obs_dis", 10, FrontObsDisCallback);
    ros::Subscriber back_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_back/obs_dis", 10, BackObsDisCallback);
    ros::Subscriber left_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_left/obs_dis", 10, LeftObsDisCallback);
    ros::Subscriber right_obs_dis_sub = nh->subscribe<std_msgs::Float32MultiArray>("/cloud_calculation_right/obs_dis", 10, RightObsDisCallback);


	TTimer can_tmr;

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		nodecheck->Find("node_rate")->Beat();
        // 转运车不在工作状态 或者在转向切换过程中
		////car_state.workmode:  0-初始化状态  1-急停  2-待机  3-工作  4-充电  5-自检  cmd_car
		if (car_state.workmode!=3 || car_state.turnmode!=car_cmd.turnmode) 
		{
			car_cmd.speed=car_cmd.angle=0;     //速度为0，转向角为0
		}

		SetWorkMode(car_cmd);   ////将夹爪和搬运车收到的控制指令控发送
		SetRunPara(car_cmd);    //发送运行参数，包括转向模式、速度，角度，充电口开/关
		SetPawMode(paw_ctr);   //发送夹爪控制模式状态，包括左右夹爪左右距离、速度，上下，取放状态
		SetObsDis();     //发送周围障碍物的距离

		ProcCanMsg();  //根据标识符can_id，发布对应的信息,包括：搬运车状态、夹爪状态、电池信息、故障码、工作距离/时间，转角

		if(nodecheck->Find("can_rate")->value>20)  can_tmr.Clear();
		else if(can_tmr.GetValue()>4)  can0_restart(), can_tmr.Clear();

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}