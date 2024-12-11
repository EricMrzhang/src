#include <c200_lidar/c200_lidar.h>
#include "common/public.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "c200_lidar_node");

    ros::NodeHandle nh("~");
    TNodeCheck* nodecheck=new TNodeCheck(&nh, "node_rate");
	nodecheck->Find("node_rate")->SetLimit(10);

    C200 lidar;
    
    while (ros::ok()) {
        ROS_INFO_STREAM("Connecting to laser at " << lidar.host_);
        lidar.connect(lidar.host_, lidar.port_);
        if (!lidar.isConnected()) {
            ROS_WARN("Unable to connect, retrying.");
            ros::Duration(1).sleep();
            continue;
        }

        ROS_DEBUG("Logging in to lidar.");
        lidar.login();
        lidar.getDeviceID();
        lidar.getSerialNumber();
        lidar.getFirmwareVersion();

		if (lidar.getDeviceState() == free_optics::device_busy) {
            lidar.disconnect();
            ROS_WARN("Device is busy. Retrying.");
            ros::Duration(1).sleep();
            continue;
        }
		else if (lidar.getDeviceState() == free_optics::device_error) {
            lidar.disconnect();
            ROS_WARN("Device is error. Retrying.");
            ros::Duration(1).sleep();
            continue;
        }
        else if (lidar.getDeviceState() == free_optics::device_high_temp_warning) {
            lidar.disconnect();
            ROS_WARN("Device high temperature warning. Retrying.");
            ros::Duration(1).sleep();
            continue;
        }
        else if (lidar.getDeviceState() == free_optics::device_high_volt_warning) {
            lidar.disconnect();
            ROS_WARN("Device high volt warning. Retrying.");
            ros::Duration(1).sleep();
            continue;
        }

        ROS_INFO("Connected to lidar.");

        
        lidar.setScanAngle();

        lidar.getScanAngle();

        lidar.scanContinous(1);

        bool result = true;
        while(ros::ok() && result == true) 
        {
            ros::spinOnce();
            result = lidar.getScanData();

            if (result == false) {
                ROS_ERROR("Lidar timed out on delivering scan, attempting to reinitialize.");
            }
            else nodecheck->Find("node_rate")->Beat();
            // ros::Duration(0.04).sleep();
        }

        // lidar.scanContinous(0);
        lidar.disconnect();

        if (ros::isShuttingDown()) {
            break;
        }
    }

    return 0;
}
