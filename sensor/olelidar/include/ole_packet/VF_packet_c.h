#pragma once
#include"ole_packet/VF_packet.h"

class VFPacket_C:public VFPacket
{
public:
#pragma pack(push, vfC, 1)
  struct Data
  {
    uint16_t angle;
    uint16_t distance;
  };
#pragma pack(pop, vfC)


public:
  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_header(uint8_t* buf);
  //virtual void read_with(PFPacketReader& reader);
};
