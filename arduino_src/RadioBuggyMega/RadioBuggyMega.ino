/**
 * @file RadioBuggyMega.ino
 * @author Haley Dalzell (haylee)
 * @author Zach Dawson (zachyzach)
 * @author Matt Sebek (msebek)
  *@author Ian Hartwig (ihartwig)
 */
#include "pinreceiver.h"
#include "brake.h"
#include "encoder.h"
#include "watchdog.h"
#include "filter.h"
#include "steering.h"
#include "rbserialmessages.h"

// Input pins
#define RX_STEERING_PIN 2
#define RX_STEERING_INT 0
#define RX_BRAKE_PIN 3
#define RX_BRAKE_INT 1
#define BRAKE_PIN 8
#define ENCODER_PIN 7

// Output pins
#define STEERING_PIN 9
#define BRAKE_INDICATOR_PIN 5
#define LED_DANGER_PIN 12

// Calibration values
#define WATCHDOG_THRESH_MS 1000
#define STEERING_CENTER 133

// Global state
unsigned long timer = 0L;
static uint8_t g_brake_state_engaged; // 0 = disengaged, !0 = engaged.
static uint8_t g_brake_needs_reset; // 0 = nominal, !0 = needs reset
RBSerialMessages g_rbserialmessages;
struct filter_state ail_state;
struct filter_state thr_state;
PinReceiver g_steering_rx;
PinReceiver g_brake_rx;

enum STATE { START, RC_CON, RC_DC, BBB_CON };

void steering_int_wrapper() {
  g_steering_rx.OnInterruptReceiver();
}

void brake_int_wrapper() {
  g_brake_rx.OnInterruptReceiver();
}

// TODO: FIX IT WHEN IT STOPS FAILING. MAKE CODE BREAK BETTER

void watchdog_fail(){
  if(g_brake_needs_reset == 0) {
    Serial1.println("Watchdog Fail! Brake dropped. Please reset brake.");
  }
  g_brake_needs_reset = 1;
}

void setup()  {
  // Initialize serial connections
  Serial1.begin(9600); // debug messages
  g_rbserialmessages.Begin(&Serial); // command/telemetry serial connection

  // init rc receiver
  g_steering_rx.Begin(RX_STEERING_PIN, RX_STEERING_INT, steering_int_wrapper);
  g_brake_rx.Begin(RX_BRAKE_PIN, RX_BRAKE_INT, brake_int_wrapper);

  filter_init(&ail_state);
  filter_init(&thr_state);
  watchdog_init(WATCHDOG_THRESH_MS, &watchdog_fail);
  brake_init(BRAKE_PIN, BRAKE_INDICATOR_PIN);
  steering_init(STEERING_PIN, 107, 126, 145);
  encoder_init(ENCODER_PIN);

  pinMode(LED_DANGER_PIN, OUTPUT);

  g_brake_state_engaged = 0; // assume disengaged
  g_brake_needs_reset = 1; // need brake reset at start
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

static int raw_angle;
static int smoothed_angle;
static int raw_thr;
static int smoothed_thr;
static int steer_angle;

void loop() {
  // find the new steering angle, if available
  if(g_steering_rx.Available()) {
    watchdog_feed();
    raw_angle = g_steering_rx.GetAngle();
    smoothed_angle = convert_rc_to_steering(raw_angle);
    steer_angle = filter_loop(&ail_state, smoothed_angle);
  }

  // find the new brake state, if available
  if(g_brake_rx.Available()) {
    watchdog_feed();
    raw_thr = g_brake_rx.GetAngle();
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
}
