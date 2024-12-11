#include "ole_packet/VF_packet_c.h"


void VFPacket_C::read_data(uint8_t *buf, size_t num)
{
  Data* data = reinterpret_cast<Data*>(buf);
  for (int i = 0; i < num; i++)
  {
    distance.push_back(data[i].distance);
    angle.push_back(data[i].angle);
  }
}

 void VFPacket_C::read_header(uint8_t * buf)
  {
    header=*reinterpret_cast<olelidar::VFHeader*>(buf);
  }
