/**
 * @file brake.h
 * @author Audrey Yeoh (ayeoh)
 * @author Matt Sebek (msebek)
 *
 * Initializes, raises, drops, and gets-state of
 * the brakes.
 *
 */
#ifndef _BRAKE_H_
#define _BRAKE_H_

  void brake_init(int brakePin, int indicatorLedPin);

  void brake_raise();

  void brake_drop();

#endif
