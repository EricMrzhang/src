#include "ole_packet/VF_packet.h"

void VFPacket::get_info()
{
    scan_info.angle_max=180;
    scan_info.angle_min=-180;
    scan_info.angle_increment=360.0/header.num_points_scan;
    scan_info.range_min=0.1;
    scan_info.range_max=50;
    scan_info.scan_time=60.0/(header.scan_frequency & 32767);
    scan_info.time_increment=scan_info.scan_time/header.num_points_scan;
    scan_info.num_size=header.num_points_scan;
    scan_info.start_index=header.first_index;
    ros::Time my_time(header.timestamp_integer+header.timestamp_decimal/4294967296.0);
    scan_info.ntp=my_time;
    scan_info.packet_size=header.num_points_packet;
}
