/**
 * @file steering.h
 * @brief Headers for Steering
 * @author ayeoh
 */

#ifndef _STEERING_H_
#define _STEERING_H_

void steering_init(int SERVO_PIN, int left, int center, int right);

void steering_set(int servo_value);

#endif /* _STEERING_H_ */
