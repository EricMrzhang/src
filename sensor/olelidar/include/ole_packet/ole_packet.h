#pragma once

#include <ros/serialization.h>

//class PFPacketReader;

struct ScanInfo
{
  float angle_min;
  float angle_max;
  float angle_increment;
  float time_increment;
  float scan_time;
  float packet_size;
  float range_min;
  float range_max;
  int num_size;
  int start_index;
  ros::Time ntp;
};


class OLEPacket
{
public:
  ros::Time last_acquired_point_stamp;
  std::vector<uint16_t> angle;
  std::vector<uint16_t> distance;
  std::vector<uint16_t> intensity;
  ScanInfo scan_info;

//   virtual void read_with(PFPacketReader& reader)
//   {
//   }
  void clear_data()
  {
    angle.clear();
    distance.clear();
    intensity.clear();
  }
  virtual void read_data(uint8_t* buf, size_t num) = 0;
  virtual void read_header(uint8_t* buf) = 0;
  virtual void get_info()=0;

  
};
