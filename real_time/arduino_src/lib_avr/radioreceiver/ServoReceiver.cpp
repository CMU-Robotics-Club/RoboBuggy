#include "ServoReceiver.h"


inline long map(long x,
                long in_offset,
                long in_scale,
                long out_offset,
                long out_scale) {
    return ((x - in_offset) * out_scale / in_scale) + out_offset;
}


ServoReceiver::ServoReceiver() {
}


void ServoReceiver::Init(volatile uint8_t *pin_reg,
                         uint8_t pin_num,
                         uint8_t int_num) {
    // save instance information
    receiver_pin_reg_ = pin_reg;
    receiver_pin_num_ = pin_num;

    // set constants
    k_min_pulse_ = 500;  // min pulse width in us
    k_max_pulse_ = 2500; // max pulse width in us
    k_offset_rc_in = 1500;
    k_scale_rc_in = 350;
    k_offset_stored_angle = 0;
    k_scale_stored_angle = 1500; // in hundredths of a degree for precision


    up_switch_time_ = 0; // us
    rc_value_ = 0;       // last recorded pulse width in us
    last_timestamp_ = 0; // us

    HardwareInit(pin_num);
}


int ServoReceiver::GetAngleHundredths() {
    // Scale the received signal into hundredths of a degree
    unsigned long temp;
    uint8_t oldSREG = SREG;

    cli();
    temp = rc_value_;
    //Return the register to its original state rather than re-enable interrupts
    SREG = oldSREG;

    int value = (int)map(temp,
                       k_offset_rc_in,
                       k_scale_rc_in,
                       k_offset_stored_angle,
                       k_scale_stored_angle);
    return value;
}


