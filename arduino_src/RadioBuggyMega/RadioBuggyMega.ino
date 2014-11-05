/**
 * @file RadioBuggyMega.ino
 * @author Haley Dalzell (haylee)
 * @author Zach Dawson (zachyzach)
 * @author Matt Sebek (msebek)
  *@author Ian Hartwig (ihartwig)
 */
#include "receiver.h"
#include "brake.h"
#include "encoder.h"
#include "watchdog.h"
#include "filter.h"
#include "steering.h"
#include "rbserialmessages.h"

// Input pins
#define BRAKE_PIN 8
#define ENCODER_PIN 7
#define THR_PIN 2
#define AIL_PIN 3

// Output pins
#define STEERING_PIN 9
#define BRAKE_INDICATOR_PIN 5
#define LED_DANGER_PIN 12

// Calibration values
#define WATCHDOG_THRESH_MS 1000
#define STEERING_CENTER 133

// Global state
unsigned long timer = 0L;
RBSerialMessages g_rbserialmessages;
struct filter_state ail_state;
struct filter_state thr_state;
static uint8_t g_brake_state_engaged; // 0 = disengaged, !0 = engaged.
static uint8_t g_brake_needs_reset; // 0 = nominal, !0 = needs reset
static int raw_angle;
static int smoothed_angle;
static int raw_thr;
static int smoothed_thr;
static int steer_angle;
static int auto_steering_angle;

enum STATE { START, RC_CON, RC_DC, BBB_CON };

// TODO: FIX IT WHEN IT STOPS FAILING. MAKE CODE BREAK BETTER


void watchdog_fail(){
  g_brake_needs_reset = 1;
  Serial1.println("Watchdog Fail! -------------------");
}


void setup()  {
  // Initialize serial connections
  Serial1.begin(9600); // debug messages
  g_rbserialmessages.Begin(&Serial); // command/telemetry serial connection

  // Initialize Buggy
  // Pins 2 and 3: pin 2 is thr, pin 3 is ail
  receiver_init();
  filter_init(&ail_state);
  filter_init(&thr_state);
  watchdog_init(WATCHDOG_THRESH_MS, &watchdog_fail);
  brake_init(BRAKE_PIN, BRAKE_INDICATOR_PIN);
  steering_init(STEERING_PIN, 107, 126, 145);
  encoder_init(ENCODER_PIN);

  pinMode(LED_DANGER_PIN, OUTPUT);

  // Init loop state
  g_brake_state_engaged = 0; // assume disengaged
  g_brake_needs_reset = 1; // need brake reset at start
  raw_angle = 0;
  smoothed_angle = 0;
  raw_thr = 0;
  smoothed_thr = 0;
  steer_angle = 0;
  auto_steering_angle = 0;
}


int convert_rc_to_steering(int rc_angle) {
  //Inverter for 2.4 GHz racecar received
  rc_angle = 180-rc_angle;
  int out = (rc_angle/4)+(90*3/4)+36;
  if(out < 100 || out > 155) {
    Serial1.println("FAKFAKFAK SERVO OUT OF RANGE");
    Serial1.println(out);
    out = 125;
  }
  return out;
}


void loop() {
  // get new command messages
  rb_message_t new_command;
  int read_status;
  while((read_status = g_rbserialmessages.Read(&new_command))
        != RBSM_ERROR_INSUFFICIENT_DATA) {
    if(read_status == 0) {
      switch(new_command.message_id) {
        case RBSM_MID_MEGA_STEER_ANGLE:
          auto_steering_angle = (int)new_command.data;
          g_rbserialmessages.Send(RBSM_MID_ERROR, new_command.data);
        default:
          // ignore unknown messages
          break;
      }
    } else if(read_status == RBSM_ERROR_INVALID_MESSAGE) {
      // g_rbserialmessages.Send(RBSM_MID_ERROR, RBSM_ERROR_INVALID_MESSAGE);
    }
    // drop messages with other faults
  }

  // find the new steering angle, if available
  if(rc_available[AIL_INDEX]) {
    watchdog_feed();
    raw_angle = receiver_get_angle(AIL_INDEX);
    smoothed_angle = convert_rc_to_steering(raw_angle);
    steer_angle = filter_loop(&ail_state, smoothed_angle);
  }

  // find the new brake state, if available
  if(rc_available[THR_INDEX]) {
    watchdog_feed();
    raw_thr = receiver_get_angle(THR_INDEX);
    smoothed_thr = filter_loop(&thr_state, raw_thr);
    // TODO make this code...less...something
    if(smoothed_thr < 70) {
      // read as engaged
      g_brake_state_engaged = 1;
      // brake has been reset
      g_brake_needs_reset = 0;
    } else {
      // read as disengaged
      g_brake_state_engaged = 0;
    }
  }

  // Always run watchdog to check if connection is lost
  watchdog_loop();

  // Set outputs
  if(g_brake_state_engaged == 0 && g_brake_needs_reset == 0) {
    brake_raise();
  } else {
    brake_drop();
  }

  steering_set(steer_angle);

  if(g_brake_needs_reset == 1) {
    digitalWrite(LED_DANGER_PIN, HIGH);
  } else {
    digitalWrite(LED_DANGER_PIN, LOW);
  }

  // Send telemetry messages
  g_rbserialmessages.Send(RBSM_MID_DEVICE_ID, RBSM_DID_DRIVE_ENCODER);
  g_rbserialmessages.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)steer_angle);
  g_rbserialmessages.Send(RBSM_MID_MEGA_BRAKE_STATE, 
                          (long unsigned)g_brake_state_engaged);
  g_rbserialmessages.Send(RBSM_MID_ERROR, 0);
}
