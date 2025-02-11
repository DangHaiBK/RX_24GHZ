#include <stdint.h>

#ifndef __RADIOLINK_H
#define __RADIOLINK_H

#define RADIOLINK_START_BYTE                0x1F
#define RADIOLINK_END_BYTE                  0x9F

#define RADIOLINK_INFO_STICK_POS            0x1A
#define RADIOLINK_INFO_BATT_STATUS          0x2A

#define RADIOLINK_STATUS_PACKET_OK          0x1C
#define RADIOLINK_STATUS_PACKET_ERR         0x2C
#define RADIOLINK_STATUS_PACKET_NO_RECV     0x3C
#define RADIOLINK_STATUS_PACKET_NO_RESP     0x4C

#define RADIOLINK_ENTER_CALIB_MODE          0x1B
#define RADIOLINK_ENTER_FORWARD_MODE        0x2B
#define RADIOLINK_ENTER_REVERSED_MODE       0x3B
#define RADIOLINK_ENTER_NEUTRAL_MODE        0x4B
#define RADIOLINK_ENTER_PARKING_MODE        0x5B
#define RADIOLINK_ENTER_SPORT_MODE          0x6B

/**
 * Define protocol struct 
 */
typedef struct {
    uint8_t startByte;
    uint8_t length;
    uint8_t infoByte;
    uint8_t payload[6];     /* 0 - throttle, 1 - yaw, 2 - pitch, 3 - roll, 4 - aux1, 5 - aux2 */
    uint8_t endByte;
} radiolink_protocol_t;

#endif  /* __RADIOLINK_H */