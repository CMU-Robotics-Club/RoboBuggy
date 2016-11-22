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
    int_num_ = int_num;

    // set constants
    k_min_pulse_ = 500;  // min pulse width in us
    k_max_pulse_ = 2500; // max pulse width in us
    k_offset_rc_in = 1500;
    k_scale_rc_in = 350;
    k_scale_steering_out = -150;
    k_offset_stored_angle = 0;
    k_scale_stored_angle = 1500; // in hundredths of a degree for precision


    up_switch_time_ = 0; // us
    rc_value_ = 0;       // last recorded pulse width in us
    last_timestamp_ = 0; // us

    // enable the necessary external interrupt
    switch(int_num_) 
    {
        case(0):
            #if defined(INT0)
            EIMSK |= _BV(INT0);
            EICRA |= _BV(ISC00);
            EICRA &= ~_BV(ISC01);
            #endif
            break;

        case(1):
            #if defined(INT1)
            EIMSK |= _BV(INT1);
            EICRA |= _BV(ISC10);
            EICRA &= ~_BV(ISC11);
            #endif
            break;

        case(2):
            #if defined(INT2)
            EIMSK |= _BV(INT2);
            EICRA |= _BV(ISC20);
            EICRA &= ~_BV(ISC21);
            #endif
            break;

        case(3):
            #if defined(INT3)
            EIMSK |= _BV(INT3);
            EICRA |= _BV(ISC30);
            EICRA &= ~_BV(ISC31);
            #endif
            break;

        case(4):
            #if defined(INT4)
            EIMSK |= _BV(INT4);
            EICRB |= _BV(ISC40);
            EICRB &= ~_BV(ISC41);
            #endif
            break;

        case(5):
            #if defined(INT5)
            EIMSK |= _BV(INT5);
            EICRB |= _BV(ISC50);
            EICRB &= ~_BV(ISC51);
            #endif
            break;

        case(6):
            #if defined(INT6)
            EIMSK |= _BV(INT6);
            EICRB |= _BV(ISC60);
            EICRB &= ~_BV(ISC61);
            #endif
            break;

        case(7):
            #if defined(INT7)
            EIMSK |= _BV(INT7);
            EICRB |= _BV(ISC70);
            EICRB &= ~_BV(ISC71);
            #endif
            break;
    }
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


