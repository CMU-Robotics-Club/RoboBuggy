#include "Dynamixel_Serial.h"
#include <avr/wdt.h>

#define TRISTATE_TOGGLE_PIN 3
#define DYNAMIXEL_BAUD 1000000
#define SERVO_ID 0xFE
#define SERVO_RECEIVE_ID 0x01 //0x04
#define CW_LIMIT_ANGLE 975
#define CCW_LIMIT_ANGLE 1425
#define NEUTRAL_ANGLE 1200
#define DEFAULT_SPEED 0x100
#define ENCODER_PIN 10

volatile long encoderTicks = 0;
actual_steering_pos = NEUTRAL_ANGLE;
desired_steering_pos = NEUTRAL_ANGLE;

void setupWatchDogTimer()
{
  // disable interupts
  cli();
  // enable wdt for every 1 second
  wdt_enable(WDTO_1S);
  // reinable interupts
  sei();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(ENCODER_PIN,INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), incrementEncoderTicks, RISING);
  Dynamixel.begin(DYNAMIXEL_BAUD);
  Dynamixel.setDirectionPin(TRISTATE_TOGGLE_PIN);
  Dynamixel.setHoldingTorque(SERVO_ID, true);
  Dynamixel.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE,CCW_LIMIT_ANGLE);
  Dynamixel.setID(SERVO_ID, SERVO_RECEIVE_ID); //Set up the Dynamixel's ID for receiving messages.
  Dynamixel.servo(SERVO_ID, desiredState.steeringPosition, DEFAULT_SPEED); //Steer to a neutral position
  //setupWatchDogTimer();
}

void loop() {
  // get desired steering from PI in i2c
  delayMicroseconds(500000);
  desired_steering_pos = desired_steering_pos == 1300 ? 1100 : 1300;
  
  if (abs(desired_steering_pos - actual_steering_pos) > 15)
    {
      //Sometimes, the user will give us crazy stuff, so we bound their desired position by our actual limits.
      int actualTargetAngle = max(CW_LIMIT_ANGLE, desired_steering_pos);
      actualTargetAngle = min(CCW_LIMIT_ANGLE, actualTargetAngle);
      Dynamixel.servo(SERVO_ID, actualTargetAngle, DEFAULT_SPEED);
      actual_steering_pos = desired_steering_pos;
    }
    //wdt_reset();
}
void incrementEncoderTicks()
{
  encoderTicks++;
}
