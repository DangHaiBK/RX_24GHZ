#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
//#define SERIAL_DEBUG     
#define BATT_CHECK_STATUS       

#define TX_ADDR                   0xAAFF122854LL
#define NUM_CHANNELS              6
#define OUTPUT_PWM 
#define SERIAL_GET_ADDR           1    
#define CHANNEL_NRF24             97   // Random channel 

#define BATT_CHECK_PERIOD         1000  // [ms]
#define SENDING_STATUS_PERIOD     100   // [ms]
#define BINDING_PERIOD            500   // [ms]
#define LOSS_TOGGLE_PERIOD        500   // [ms]
#define FRAME_RATE                10    // [ms]
#define TOGGLE_BINDING_TIME       500   // [ms]
#define SIGNAL_LOST_TIMEOUT       1000  // [ms]
#define SIGNAL_FEEDBACK_TIMEOUT   2000  // [ms]

#define VALUE_ADC_MIN             0
#define VALUE_ADC_MID             511
#define VALUE_ADC_MAX             1023

#define VALUE_STICK_MIN           1000
#define VALUE_STICK_MID           1500
#define VALUE_STICK_MAX           2000

#define BATT_VOLT_THRES           2.7f
#define VALUE_DIFFERENCE          10

#define CE_PIN                    9
#define CSN_PIN                   10      

#define LED_POWER                 8
#define LED_STATUS_RX             11

#define BUTTON_BINDING            5

#define BATT_VOLT_PIN             2

#define ROLL_CHANNEL              3
#define YAW_CHANNEL               4
#define PITCH_CHANNEL             5
#define THROTTLE_CHANNEL          6

#define AUX1_CHANNEL              7
#define AUX2_CHANNEL              8

/**
 * Define channel data struct
 */
typedef struct {
    uint8_t throttle;
    uint8_t yaw;
    uint8_t pitch;
    uint8_t roll;
    uint8_t AUX1;
    uint8_t AUX2;
} dataPacket;

/**
 * Define pseudo channel data struct
 */
typedef struct {
    uint8_t yaw;
    uint8_t pitch;
    uint8_t roll;
} pseudo_packet_t;

#endif /* __CONFIG_H */