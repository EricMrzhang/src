#include "ole_packet/VF_packet_a.h"


void VFPacket_A::read_data(uint8_t *buf, size_t num)
{
  Data* data = reinterpret_cast<Data*>(buf);
  for (int i = 0; i < num; i++)
  {
    distance.push_back(data[i].distance);
  }
}

void VFPacket_A::read_header(uint8_t *buf)
{
   header=*reinterpret_cast<olelidar::VFHeader*>(buf);
}
// void VFPacket_A::read_with(PFPacketReader &reader)
// {
//     reader.read(*this);
// }
