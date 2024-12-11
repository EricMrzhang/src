#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include "ole_packet/ole_packet.h"
#include "handle_info.h"
#include <thread>
#include <cmath>
#include"communication/udp_transport.h"
#include"communication/tcp_transport.h"
#include"ole_packet/VF_packet_a.h"
#include"ole_packet/VF_packet_b.h"
#include"ole_packet/VF_packet_c.h"

class VFPacket_A;
class VFPacket_B;
class VFPacket_C;

using namespace sensor_msgs;

namespace olelidar
{
    class Ros_node
    {
    public:
        Ros_node(const ros::NodeHandle &pnh);
        bool OpenPort();
        void ReadPacket();
        void GetDataType(uint8_t* data);
        void FillData();
        void Publish();
        void SelectType(uint8_t* data);
        ~Ros_node();
    private:
        float deg2rad(float deg) {
           return deg * M_PI / 180.0;
        }
        ros::Publisher scan_pub_;
        ros::Time last_time_;
        ros::NodeHandle pnh_;
        sensor_msgs::LaserScan scanData;
        std::string device_ip_str_;
        bool is_loop=true;
        int seq;
        int device_port_;
        int data_type;
        size_t packet_size;
        std::string product_;
        std::string local_ip_str_;
        std::string transport_str_;  
        std::string frame_id_;     
        std::unique_ptr<Transport> transport_;
        std::shared_ptr<OLEPacket> parser;
        std::shared_ptr<HandleInfo> info;
        std::thread reader;
        bool use_ntp;
        uint16_t current_angle;
        uint16_t last_angle;
        boost::array<uint8_t, 4096> buf;
    };
}
