/**
 * @file receiver.cpp
 * @brief Contains Code for dealing with RC Receiver
 */
//#include <stdint.h>
#include <Arduino.h>
#include "receiver.h"


#define AIL_LEFTMOST 2000
#define AIL_RIGHTMOST 980
#define AIL_CENTERMOST 1480

#define THR_LEFTMOST -1
#define THR_RIGHTMOST -1
#define THR_CENTERMOST -1

#define AIL_RECEIVER_PIN 3
#define AIL_RECEIVER_INT 1

#define THR_RECEIVER_PIN 2
#define THR_RECEIVER_INT 0

//#define THR_INDEX 0
//#define AIL_INDEX 1

// Note: arr[0] is thr, arr[1] is ail
static volatile int start_time[2];
static volatile int rc_value[2];


// When our code gets really busy this will become inaccurate,
// (i believe since micros gets shifted a bit) but for
// the current application its easy to understand and works very well
// TODO if things start twitching, move to using registers directly.
static void receiver_on_ail_interrupt() {
  if(digitalRead(AIL_RECEIVER_PIN) == HIGH) {
    start_time[AIL_INDEX] = micros();
  } else {
    if(start_time[AIL_INDEX] && (rc_available[AIL_INDEX] == 0)) {
      rc_value[AIL_INDEX] = (int)(micros() - start_time[AIL_INDEX]);
      start_time[AIL_INDEX] = 0;
      rc_available[AIL_INDEX] = 1;
    }
  }
}

static void receiver_on_thr_interrupt() {
  if(digitalRead(THR_RECEIVER_PIN) == HIGH) {
    start_time[THR_INDEX] = micros();
  } else {
    if(start_time[THR_INDEX] && (rc_available[THR_INDEX] == 0)) {
      rc_value[THR_INDEX] = (int)(micros() - start_time[THR_INDEX]);
      start_time[THR_INDEX] = 0;
      rc_available[THR_INDEX] = 1;
    }
  }
}

// Returns error code
int receiver_init() {
  attachInterrupt(THR_RECEIVER_INT, receiver_on_thr_interrupt, CHANGE);
  attachInterrupt(AIL_RECEIVER_INT, receiver_on_ail_interrupt, CHANGE);
}


// Index = 0 to check thr, index = 1 to check
// Returns 0 to 180, with 90 being center.
// TODO measure throttle positions.
int receiver_get_angle(int index) {
  // Math to convert nThrottleIn to 0-180.
  int ret_val = (rc_value[index]-AIL_RIGHTMOST)*3/17;
  rc_available[index] = 0;
  return ret_val;
}

