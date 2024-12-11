#pragma once
#include"ole_packet/VF_packet.h"

class VFPacket_B:public VFPacket
{
public:
#pragma pack(push, vfB, 1)
  struct Data
  {
    uint16_t distance;
    uint16_t intensity;
  };
#pragma pack(pop, vfB)

public:
  virtual void read_data(uint8_t* buf, size_t num);
  virtual void read_header(uint8_t* buf);
  //virtual void read_with(PFPacketReader& reader);
};