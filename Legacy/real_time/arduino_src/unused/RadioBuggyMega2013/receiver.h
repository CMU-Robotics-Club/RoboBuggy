/**
 * @file receiver.h
 * @brief Headers for Receiver
 * DOGE mooooooo
 * doge COW!
 */

#ifndef _RECEIVER_H_
#define _RECEIVER_H_

#ifdef __cplusplus
extern "C"{
#endif

  #define THR_INDEX 0
  #define AIL_INDEX 1

  // To check if new value available, check the
  // value contained in this, using the correct
  // index.
  volatile int rc_available[2];

  // if this is true, we are connected.
  volatile char rc_connected;

  int receiver_init();

  // Zero for throttle position, 1 for ail pos
  // returns an angle strictly between 0 and 180,
  // with 90 being the center position.
  int receiver_get_angle(int int_pin);

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* _RECEIVER_H_ */

