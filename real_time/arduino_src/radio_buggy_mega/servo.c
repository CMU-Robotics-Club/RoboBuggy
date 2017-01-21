#include "servo.h"


void servo_init(void) {
    // set up timers for PWM on OC1A (PB1, arduino pin 9)
    // prescaler of 8
    // PWM with top as ICR1 (to set cap)
    // set ICR1 to the desired refresh time
    TCCR1A |= (_BV(WGM11) | _BV(COM1A1));
    TCCR1A &= ~(_BV(WGM10) | _BV(COM1A0));
    TCCR1B |= (_BV(WGM13) | _BV(WGM12) | _BV(CS11));
    TCCR1B &= ~(_BV(CS10) | _BV(CS12));
    TCNT1 = 0;
    servo_set_us(SERVO_MID_US);
    ICR1 = microsecondsToClockCycles(SERVO_REFRESH_US) / SERVO_TIMER_PRESCALER;

    // enable servo output
    SERVO_PORT &= ~_BV(SERVO_PINN);
    SERVO_DDR |= _BV(SERVO_PINN);
}


void servo_set_us(uint16_t value) {
    // check limits
    if(value < SERVO_MIN_US) {
        value = SERVO_MIN_US;
    }
    if(value > SERVO_MAX_US) {
        value = SERVO_MAX_US;
    }

    // modify hardware output
    OCR1A = microsecondsToClockCycles(value) / SERVO_TIMER_PRESCALER;
}
