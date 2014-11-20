/**
 * @file steering.c
 * @brief These functions are used to control the steering servo motor
 * @author Haley Dalzell
 * @author Zach Dawson
 * @author Matt Sebek
 *
 */
#include <Servo.h>

Servo steer;  // create servo object to control a servo
static int s_angle;
static int s_left, s_center, s_right;

void steering_init(int SERVO_PIN, int left, int center, int right) {
  steer.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  s_left = left;
  s_center = center;
  s_right = right;
}

void steering_set(int servo_value) {
  //s_angle = servo_value;
  if(servo_value < s_left) {
    s_angle = s_left;
  } else if(servo_value > s_right) {
    s_angle = s_right;
  } else {
    s_angle = servo_value;
  }
  steer.write(s_angle);
}
