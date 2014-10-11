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


#define BRAKE_PIN 8
#define BRAKE_INDICATOR_PIN 5

#define ENCODER_PIN 7

#define STEERING_PIN 9
#define STEERING_CENTER 133

#define THR_PIN 2
#define AIL_PIN 3

#define TIME_THRESH 1000

unsigned long timer = 0L;
static unsigned long last_time;
static uint8_t g_brake_state; // 0 = engaged, !0 = disengaged.
RBSerialMessages g_rbserialmessages;

// Initialize filter for
struct filter_state ail_state;
struct filter_state thr_state;

#define LED_DANGER_PIN 12

enum STATE { START, RC_CON, RC_DC, BBB_CON };

// TODO: FIX IT WHEN IT STOPS FAILING. MAKE CODE BREAK BETTER

void watchdog_fail(){
 brake_drop();
 Serial1.println("Watchdog Fail! -------------------");
 digitalWrite(LED_DANGER_PIN, HIGH);
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
  watchdog_init(TIME_THRESH, &watchdog_fail);
  brake_init(BRAKE_PIN, BRAKE_INDICATOR_PIN);
  steering_init(STEERING_PIN, 107, 126, 145);
  encoder_init(ENCODER_PIN);

  pinMode(LED_DANGER_PIN, OUTPUT);

  // Set up loop
  //last_time = millis();

  g_brake_state = 1; // assume disengaged
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

//code that keeps loop time constant each loop
static int hz = 40;
static int print_period = 1000 / hz;

static int raw_angle;
static int smoothed_angle;
static int raw_thr;
static int smoothed_thr;
static int steer_angle;

void loop() {
  //if(filter_loop(&thr_state, rc_available[THR_INDEX])) {
  if(rc_available[AIL_INDEX]) {
    watchdog_feed();
    raw_angle = receiver_get_angle(AIL_INDEX);
    smoothed_angle = convert_rc_to_steering(raw_angle);
    steer_angle = filter_loop(&ail_state, smoothed_angle);
    steering_set(steer_angle);
  }

  //if(filter_loop(&ail_state, rc_available[AIL_INDEX])) {
  if(rc_available[THR_INDEX]) {
    watchdog_feed();
    raw_thr = receiver_get_angle(THR_INDEX);
    smoothed_thr = filter_loop(&thr_state, raw_thr);
    // TODO make this code...less...something
    if(smoothed_thr < 70) {
      g_brake_state = 1;
      brake_drop();
    } else {
      g_brake_state = 0;
      brake_raise();
    }
  }

  // Loop
  watchdog_loop();
  encoder_loop();

  // If timer expired, then do ROS things
  if((last_time - millis()) > 0) {
    Serial1.print(steer_angle);
    Serial1.print("    ");
    Serial1.println(encoder_get_count());
  }

  // Send telemetry messages
  g_rbserialmessages.Send(RBSM_MID_DEVICE_ID, RBSM_DID_DRIVE_ENCODER);
  g_rbserialmessages.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)steer_angle);
  g_rbserialmessages.Send(RBSM_MID_MEGA_BRAKE_STATE, 
                          (long unsigned)g_brake_state);

}
