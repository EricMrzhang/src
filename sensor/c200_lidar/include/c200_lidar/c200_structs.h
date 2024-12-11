#ifndef C200_STRUCTS_H_
#define C200_STRUCTS_H_

#include <stdint.h>

namespace free_optics {

struct scanConfig
{
    int scan_frequency;

    int angle_resolution;

    int start_angle;

    int stop_angle;
};

struct scanOutputRange
{
    int angle_resolution;

    int start_angle;

    int stop_angle;
};

struct scanData
{
    int dist_len;

    uint16_t dist_data[1351];

    uint16_t rssi_data[1351];
};

typedef enum
{
    device_busy = 0,
    device_ready = 1,
    device_error = 2,
    device_high_temp_warning = 3,
    device_high_volt_warning = 4,
    device_temp_compensation = 5,
    other_status = 6
} status_t;

#pragma pack(1)
struct packetHeader
{
    uint8_t header[4];
    uint16_t packet_size;
    uint8_t opt_code;
    uint8_t cmd_code;
    uint8_t device_state;
    uint16_t scan_number;
    uint8_t packet_number;
    uint16_t scan_frequency;
    uint16_t angle_resolution;
    uint16_t points_per_scan;
    uint16_t angle_index;
    uint16_t points_per_packet;
};
#pragma pack()

}

#define READ_DOWN           0x00
#define WRITE_DOWN          0x01
#define METHOD_DOWN         0x02
#define READ_UP             0x10
#define WRITE_UP            0x11
#define METHOD_UP           0x12

#define RESET_DEV           0
#define LOGIN               1
#define GET_DEV_ID          10
#define GET_DEV_STA         11
#define GET_FW_VER          12
#define GET_SN              24
#define SET_ANGLE			27
#define GET_ANGLE           28
#define LOOP_SWITCH         49
#define LOOP_DATA           50

#endif
