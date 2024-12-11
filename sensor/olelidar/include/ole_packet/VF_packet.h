#pragma once
#include"ole_packet/ole_packet.h"
#include<olelidar/VFHeader.h>

class VFPacket : public OLEPacket
{
public:
    virtual void get_info();
    olelidar::VFHeader header;
};