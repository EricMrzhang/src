#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <common/public.h>

using namespace std;


char* SERVER_IP = "192.168.4.7";
int MYPORT=502;      //10001;  //端口号

int socket_cli=0;
int datacount=10;
unsigned char datavalue[1000];

ros::NodeHandle *nh;
ros::Publisher vin_pub;
TNodeCheck *nodecheck;
 
void Send(char *str, int count)
{
    struct sockaddr_in sendaddr;

    sendaddr.sin_family = AF_INET;
    sendaddr.sin_port = htons(MYPORT);
    sendaddr.sin_addr.s_addr = inet_addr(SERVER_IP);
    sendto(socket_cli, str, count, 0, (struct sockaddr *)&sendaddr, sizeof(struct sockaddr_in));
}

void RecvData()
{
	unsigned char recvbuf[1000];
	int n=recv(socket_cli, recvbuf, sizeof(recvbuf), 0);

	if(recvbuf[6]==0x01 && recvbuf[7]==0x03 && recvbuf[8]==datacount*2)
    {
        nodecheck->Find("rfid_rate")->Beat();
		if(recvbuf[9]!=0xff)
		{
			char vinstr[500]={0};   
			for(int i=0;i<12;i++)  sprintf(vinstr,"%s%02x",vinstr, recvbuf[i+9]);
			nh->setParam("cargoVin", vinstr);

			std_msgs::String msg;
			msg.data=vinstr;
			vin_pub.publish(msg);
		}
		// ROS_INFO(vinstr);
    }
}

int InitTcp()
{
	socket_cli = socket(AF_INET, SOCK_STREAM, 0);
	if(socket_cli < 0)
	{
		std::cout << "socket() error\n";
		return 0;
	}
	
	struct sockaddr_in sev_addr;  
	memset(&sev_addr, 0, sizeof(sev_addr));
	sev_addr.sin_family      = AF_INET;
	sev_addr.sin_port        = htons(MYPORT);
	sev_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
	std::cout << "connecting..." << std::endl;
	if(connect(socket_cli, (struct sockaddr*) &sev_addr, sizeof(sev_addr)) < 0)
	{ 
		std::cout << "connect error" << std::endl;
		return 0;
	}
	else
	{
		std::cout << "connected successfullly!" << std::endl;
		return 1;
	}	
}
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "rfid_reader");
	nh=new ros::NodeHandle("~");
	vin_pub = nh->advertise<std_msgs::String>("vin", 10);
	nodecheck=new TNodeCheck(nh, "rfid_rate");

	if(!InitTcp()) return 0;
	
	while(ros::ok())
	{
		unsigned char buf[100]={0x00, 0x00, 00, 00, 00, 0x06, 0x01, 0x03, 0x02, 0x01, 00, datacount};
        Send((char*)buf, 12);
        usleep(400000);
        
        RecvData();
	}

	close(socket_cli);
	return 0;
}