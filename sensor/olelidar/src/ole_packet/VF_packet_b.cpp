#include "ole_packet/VF_packet_b.h"



void VFPacket_B::read_data(uint8_t *buf, size_t num)
{
  Data* data = reinterpret_cast<Data*>(buf);
  for (int i = 0; i < num; i++)
  {
    distance.push_back(data[i].distance);
    intensity.push_back(data[i].intensity);
  }
}

  void VFPacket_B::read_header(uint8_t * buf)
  {
    header=*reinterpret_cast<olelidar::VFHeader*>(buf);
  }
