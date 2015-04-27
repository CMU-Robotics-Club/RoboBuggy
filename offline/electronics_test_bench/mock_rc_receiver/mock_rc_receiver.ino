#include <Servo.h>

#define RC_STEERING_PIN 9
#define RC_BRAKE_PIN 10
#define RC_AUTON_PIN 11
#define POT_STEETING_PIN A0
#define POT_BRAKE_PIN A1
#define SWITCH_AUTON_PIN 8
#define DEBUG_LED_PIN 13

#define SERVO_MIN_US 544
#define SERVO_MAX_US 2400

Servo g_steering_servo;
Servo g_brake_servo;
Servo g_auton_servo;


void setup() {
  // configure outputs
  g_steering_servo.attach(RC_STEERING_PIN);
  g_brake_servo.attach(RC_BRAKE_PIN);
  g_auton_servo.attach(RC_AUTON_PIN);
  
  // configure inputs
  pinMode(POT_STEETING_PIN, INPUT);
  pinMode(POT_BRAKE_PIN, INPUT);
  pinMode(SWITCH_AUTON_PIN, INPUT_PULLUP);
  
  // let the world know we are running
  pinMode(DEBUG_LED_PIN, OUTPUT);
  digitalWrite(DEBUG_LED_PIN, HIGH);
}


uint16_t analog_to_servo(uint8_t analog_pin) {
  uint16_t setpoint = analogRead(analog_pin)/4;
  return map(setpoint, 0, 0xFF, SERVO_MIN_US, SERVO_MAX_US);
}


void loop() {
  g_steering_servo.writeMicroseconds(analog_to_servo(POT_STEETING_PIN));
  g_brake_servo.writeMicroseconds(analog_to_servo(POT_BRAKE_PIN));
  
  if(digitalRead(SWITCH_AUTON_PIN)) {
    // send max value
    g_auton_servo.writeMicroseconds(SERVO_MAX_US);
  } else {
    g_auton_servo.writeMicroseconds(SERVO_MIN_US);
  }
}
