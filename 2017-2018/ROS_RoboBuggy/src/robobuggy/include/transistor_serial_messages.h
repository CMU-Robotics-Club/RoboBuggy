#ifndef _TRANSISTOR_SERIAL_MESSAGES_H_
#define _TRANSISTOR_SERIAL_MESSAGES_H_

// Protocol constants
#define RBSM_FOOTER     0x0A // \n
#define RBSM_IN_LENGTH  6

// Message Types
#define RBSM_MID_ENC_TICKS_RESET            1
#define RBSM_MID_ENC_RESET_REQUEST          5
#define RBSM_MID_ENC_RESET_CONFIRM          6
#define RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND  17
#define RBSM_MID_MEGA_AUTON_BRAKE_COMMAND   18
#define RBSM_MID_MEGA_STEER_COMMAND         19
#define RBSM_MID_MEGA_STEER_ANGLE           20
#define RBSM_MID_MEGA_BRAKE_STATE           21
#define RBSM_MID_MEGA_AUTON_STATE           22
#define RBSM_MID_MEGA_BATTERY_LEVEL         23
#define RBSM_MID_MEGA_STEER_FEEDBACK        24
#define RBSM_MID_RESERVED                   252 // 0xFC, message head
#define RBSM_MID_ERROR                      254
#define DEVICE_ID                           255
#define FOOTER                              10

#endif /* _TRANSISTOR_SERIAL_MESSAGES_H_ */
