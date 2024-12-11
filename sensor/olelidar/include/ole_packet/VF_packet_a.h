#pragma once
#include"ole_packet/VF_packet.h"

class VFPacket_A:public VFPacket
{
public:
#pragma pack(push, vfA, 1)
  struct Data
  {
    uint16_t distance;
  };
#pragma pack(pop, vfA)

public:
  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_header(uint8_t* buf);
  //virtual void read_with(PFPacketReader& reader);
};
