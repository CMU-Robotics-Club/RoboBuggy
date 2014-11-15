/**
 * @file RadioBuggyMega.ino
 * @author Haley Dalzell (haylee)
 * @author Zach Dawson (zachyzach)
 * @author Matt Sebek (msebek)
 */
 #include <Arduino.h>
#include <Servo.h>
#include "receiver.h"
#include "brake.h"
#include "encoder.h"
#include "steering.h"
#include "rbserialmessages.c"
#include "rbserialmessages.h"

// Before compiling switch device to Arduino Mega
// Read http://www.arduino.cc/en/Hacking/BuildProcess before doing work. 
//   Because Arduino is not a real build system. 
#define BRAKE_PIN 8
#define BRAKE_INDICATOR_PIN 5

#define ENCODER_PIN 7

#define STEERING_PIN 9
#define STEERING_CENTER 133

#define THR_PIN 2
#define AIL_PIN 3

#define XBEE_MSG_REC 4

#define XBEE_DANGER 12

static unsigned long last_time = 0;

unsigned long timer = 0L;
static char data; // used to pass things into xbee
int brake = 0;


enum STATE { START, RC_CON, RC_DC, BBB_CON };

void setup()  {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(XBEE_MSG_REC, OUTPUT);

  // Initialize Buggy
  // Pins 2 and 3: pin 2 is thr, pin 3 is ail
  receiver_init();
  brake_init(BRAKE_PIN, BRAKE_INDICATOR_PIN);
  steering_init(STEERING_PIN, 133, 115, 160);
  encoder_init(ENCODER_PIN);

  // Set up loop
  last_time = millis();
}

int convert_rc_to_steering(int rc_angle) {
  int out = (rc_angle/4)+(90*3/4)+39;
  if(out < 105 || out > 160) {
    Serial.println("FAKFAKFAK SERVO OUT OF RANGE");
    Serial.println(out);
    out = 129;
  }
}

//code that keeps loop time constant each loop
static int hz = 40;
static int print_period = 1000 / hz;

static int rc_angle;
static int rc_thr;
void loop() {

  if(rc_available[THR_INDEX]) {
    rc_angle = receiver_get_angle(THR_INDEX);
  }

  steering_set(convert_rc_to_steering(rc_angle));

  if(rc_available[AIL_INDEX]) {
    rc_thr = receiver_get_angle(THR_PIN);
  }

  // Loop
  encoder_loop();
  //xbee_loop();

  // If timer expired, then do ROS things
  if((last_time - millis()) > 0) {
    Serial.println("Loop!");


  }

}
