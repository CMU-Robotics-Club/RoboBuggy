#ifndef LIB_AVR_SERVO_H
#define LIB_AVR_SERVO_H

#include <avr/io.h>


// library settings
#define SERVO_REFRESH_US 20000L
#define SERVO_MID_US 1500
#define SERVO_MIN_US 544
#define SERVO_MAX_US 2400
#define SERVO_TIMER_PRESCALER 8

#define SERVO_DDR  DDRB
#define SERVO_PORT PORTB
#define SERVO_PINN PB5 // arduino 11


// useful macros
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )


// public functions
void servo_init(void);
void servo_set_us(uint16_t value);

#endif /* LIB_AVR_SERVO_H */
