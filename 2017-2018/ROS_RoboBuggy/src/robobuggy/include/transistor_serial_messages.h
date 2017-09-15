#ifndef _TRANSISTOR_SERIAL_MESSAGES_H_
#define _TRANSISTOR_SERIAL_MESSAGES_H_

// Port and baud rate
#define RBSM_SERIAL_PORT "/dev/ttyACM0"
#define RBSM_SERIAL_BAUD 115200

// Protocol constants
#define RBSM_FOOTER     0x0A // \n
#define RBSM_IN_LENGTH  6

// Message Types
#define RBSM_MID_ENC_TICKS_RESET    1
#define RBSM_MID_ENC_TIMESTAMP      2
#define RBSM_MID_MEGA_STEER_ANGLE   20
#define RBSM_MID_MEGA_BRAKE_STATE   21
#define RBSM_MID_MEGA_AUTON_STATE   22
#define RBSM_MID_MEGA_BATTERY_LEVEL 23
#define RBSM_MID_RESERVED           252 // 0xFC, message head
#define RBSM_MID_ERROR              254
#define RBSM_MID_DEVICE_ID          255

#endif /* _TRANSISTOR_SERIAL_MESSAGES_H_ */
