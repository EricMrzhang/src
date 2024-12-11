#ifndef C200_LIDAR_H_
#define C200_LIDAR_H_

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <ros/ros.h>
#include <string>
#include <stdint.h>
#include <c200_lidar/c200_structs.h>
#include <sensor_msgs/LaserScan.h>

using namespace free_optics;

class C200
{
public:
    C200();
    virtual ~C200();

    void connect(std::string host, int port = 2111);

    void disconnect();

    bool isConnected();

    void login();

    void getDeviceID();

    void getSerialNumber();

    void getFirmwareVersion();

    status_t getDeviceState();

    void getScanAngle();
    
    void setScanAngle();

    void scanContinous(uint8_t state);

    bool getScanData();

    void saveConfig();
	void filterHandle(std::vector<float>, std::vector<float>, int);

    std::string host_;
    int port_;
	bool filterswitch_;

protected:
    bool connected_;
    int socket_fd_;
    uint8_t buffer[65536];

private:
    uint16_t byteExchange(uint16_t value);

    ros::NodeHandle nh_, nh_private_;
    ros::Publisher laser_scan_publisher_;
    sensor_msgs::LaserScan laser_scan;

    std::string frame_id_;
    double angle_min_;
    double angle_max_;
    double range_min_;
    double range_max_;
    int offset_angle_;
    int32_t start_angle_;
    int32_t stop_angle_;
};

#endif
