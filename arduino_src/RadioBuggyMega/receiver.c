/**
 * @file receiver.c
 * @brief Contains Code for dealing with RC Receiver
 * NOTE: AIL and THR are flipped.
 * AIL is on pin 2
 * THR is on pin 3
 *
 * @author Matt Sebek (msebek)
 * @author Zach Dawson (zsd)
 *
 * These functions are used to connect/communicate with
 * the RC radio receiver.
 */
#include <Arduino.h>
#include "receiver.h"

#define AIL_LEFTMOST 2000
#define AIL_RIGHTMOST 980
#define AIL_CENTERMOST 1480

#define THR_LEFTMOST -1
#define THR_RIGHTMOST -1
#define THR_CENTERMOST -1

#define AIL_RECEIVER_PIN 2
#define AIL_RECEIVER_INT 0

#define THR_RECEIVER_PIN 3
#define THR_RECEIVER_INT 1

//For 72 MHz RC reciever
//#define PWM_TIME 21800
//#define PWM_THRESH 130
//#define BIG_PULSE 10
//High pulse lasts over 1/10 of period
//#define SHORT_PULSE 21
//High pulse lasts under 1/21 of period

//For 2.4 GHz racecar reciever
#define PWM_TIME 18370   //the pwn period (us)
#define PWM_THRESH 1300
#define BIG_PULSE 8
//High pulse lasts over 1/8 of period
#define SHORT_PULSE 19
//High pulse lasts under 1/19 of period

// Defined in receiver.h
// #define THR_INDEX 0
// #define AIL_INDEX 1

// Note: arr[0] is thr, arr[1] is ail

static volatile unsigned long up_switch_time[2];
static volatile unsigned long down_switch_time[2];
static volatile unsigned long rc_value[2];

// NOTE THAT WE ARE ASSUMING:
//  Main Loop-length is shorter than PWM pulse length. 
//    Otherwise, you could recieve stale values in your main loop.
// Loop reading rc_values must set rc_available false after reading
// When our code gets really busy this will become inaccurate,
// (i believe since micros gets shifted a bit) but for
// the current application its easy to understand and works very well
// TODO if things start twitching, move to using registers directly.
// TODO if this starts twitching, also micros has a resolution of 4us.
static void receiver_on_ail_interrupt() {
  if(digitalRead(AIL_RECEIVER_PIN) == HIGH) {
    // High Received
    if((micros() - up_switch_time[AIL_INDEX] > PWM_TIME - PWM_THRESH) &&
       (micros() - up_switch_time[AIL_INDEX] < PWM_TIME + PWM_THRESH)) {
      // TODO this prevent an instantaneous start-up error, where the
      // above condition is true, and up_switch_time 
      if((down_switch_time[AIL_INDEX] > up_switch_time[AIL_INDEX]) &&	
	 (rc_available[AIL_INDEX] == 0)) {
	  rc_value[AIL_INDEX] = (down_switch_time[AIL_INDEX] - 
				 up_switch_time[AIL_INDEX]);
	  rc_available[AIL_INDEX] = 1;
          if(rc_value[AIL_INDEX]/(PWM_TIME/BIG_PULSE) >= 1 || rc_value[AIL_INDEX]/(PWM_TIME/SHORT_PULSE) == 0)
            rc_available[AIL_INDEX] == 0;
      }
    }
    up_switch_time[AIL_INDEX] = micros();
  } else {
    // Low received
    if(up_switch_time[AIL_INDEX]) {
      down_switch_time[AIL_INDEX] = micros();
    }
  }
}

static void receiver_on_thr_interrupt() {
  if(digitalRead(THR_RECEIVER_PIN) == HIGH) {
    // High Received
    if((micros() - up_switch_time[THR_INDEX] > PWM_TIME - PWM_THRESH) &&
       (micros() - up_switch_time[THR_INDEX] < PWM_TIME + PWM_THRESH)) {
      // TODO this prevent an instantaneous start-up error, where the
      // above condition is true, and up_switch_time 
      if((down_switch_time[THR_INDEX] > up_switch_time[THR_INDEX]) &&	
	 (rc_available[THR_INDEX] == 0)) {
	  rc_value[THR_INDEX] = (down_switch_time[THR_INDEX] - 
				 up_switch_time[THR_INDEX]);
	  rc_available[THR_INDEX] = 1;
          if(rc_value[THR_INDEX]/(PWM_TIME/10) >= 1 || rc_value[THR_INDEX]/(PWM_TIME/21) == 0)
            rc_available[THR_INDEX] == 0;
      }
    }
    up_switch_time[THR_INDEX] = micros();
  } else {
    // Low received
    if(up_switch_time[THR_INDEX]) {
      down_switch_time[THR_INDEX] = micros();
    }
  }
}

// Returns error code
int receiver_init() {
  up_switch_time[0] = 0;
  up_switch_time[1] = 0;
  down_switch_time[0] = 0;
  down_switch_time[1] = 0;
  attachInterrupt(THR_RECEIVER_INT, receiver_on_thr_interrupt, CHANGE);
  attachInterrupt(AIL_RECEIVER_INT, receiver_on_ail_interrupt, CHANGE);
}


// Index = 0 to check thr, index = 1 to check
// Returns 0 to 180, with 90 being center.
// TODO measure throttle positions.
int receiver_get_angle(int index) {
  // Math to convert nThrottleIn to 0-180.
  int ret_val = (int)(rc_value[index]-AIL_RIGHTMOST)*3/17;
  rc_available[index] = 0;
  return ret_val;
}




