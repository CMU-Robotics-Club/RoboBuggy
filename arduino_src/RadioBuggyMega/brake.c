/**
 * @file brake.c
 * @author Audrey Yeoh (ayeoh)
 * @author Matt Sebek (msebek)
 */
#include <Arduino.h>
#include "brake.h"

static int brake_pin;
// Indicator LED pin number
static int indicator_pin;

void brake_init(int brakePin, int indicatorLed) {
  pinMode(brakePin, OUTPUT);
  pinMode(indicatorLed, OUTPUT);
  brake_pin = brakePin;
  indicator_pin = indicatorLed;
}

// Note: High-voltage raises the brake.
// Raises the brake
// Do not call before brake_init
void brake_raise() {
  digitalWrite(brake_pin, HIGH);
  digitalWrite(indicator_pin, LOW);
}

// Drops the brake
// Do not call before brake_init
void brake_drop() {
  digitalWrite(brake_pin, LOW);
  digitalWrite(indicator_pin, HIGH);
}
