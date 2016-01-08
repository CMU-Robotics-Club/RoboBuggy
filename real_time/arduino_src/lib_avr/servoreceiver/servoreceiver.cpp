#include "servoreceiver.h"


inline long map(long x,
                long in_offset,
                long in_scale,
                long out_offset,
                long out_scale) 
{
    return ((x - in_offset) * out_scale / in_scale) + out_offset;
}


ServoReceiver::ServoReceiver()
{
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
    k_offset_steering_out = 1789;
    k_scale_steering_out = -150;
    k_offset_stored_angle = 0;
    k_scale_stored_angle = 1000; // in hundredths of a degree for precision


    up_switch_time_ = 0; // us
    rc_value_ = 0;       // last recorded pulse width in us
    last_timestamp_ = 0; // us

    // enable the necessary external interrupt
    switch(int_num_) {
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


/*
no arguments
Catches an incoming edge (up or down) in order to time it. This function should
should be called by the appropriate ISR() in the top level file.
*/
void ServoReceiver::OnInterruptReceiver(){
    uint8_t new_pin_value = (*receiver_pin_reg_) & _BV(receiver_pin_num_);
    unsigned long new_timestamp = micros();
    
    if (new_pin_value != 0){
        // record the start of a suspected pulse
        up_switch_time_ = micros();
    }

    else {
        // check that we actually recorded the beginning of this pulse
        if (up_switch_time_ != 0) {
            // check pulse width is within normal range
            unsigned long pulse_width = new_timestamp - up_switch_time_;
            if(pulse_width > k_min_pulse_ && pulse_width < k_max_pulse_) {
                // save ok values
                rc_value_ = pulse_width;
                last_timestamp_ = new_timestamp;
            }

        }
    }
}


/*
no arguments
Returns the timestamp of the last successfully received servo pulse in
microseconds since startup or last overflow.
*/
unsigned long ServoReceiver::GetLastTimestamp() {
    return last_timestamp_;
}


/*
no arguments
Return the last read pulse width in microseconds.
*/
unsigned long ServoReceiver::GetPulseWidth() {
    return rc_value_;
}


/*
no arguments
Returns the angle measurement of the last read pulse width. As far as I can
tell, this is an arbitrary scale and has an arbitrary offset.
*/
int ServoReceiver::GetAngle(){
    int ret_val = (int)(rc_value_ - AIL_RIGHTMOST)*3/17; //WHYYYYY
    return ret_val;
}


int ServoReceiver::GetAngleThousandths() {
  // Scale the received signal into hundredths of a degree
  int value = (int)map(rc_value_,
                       k_offset_rc_in,
                       k_scale_rc_in,
                       k_offset_stored_angle,
                       k_scale_stored_angle);
  return value;
}


void ServoReceiver::PrintDebugInfo(FILE *out_stream){
    fprintf(out_stream, "up_switch_time: %lu\n", up_switch_time_);
    fprintf(out_stream, "last_timestamp: %lu\n", last_timestamp_);
    fprintf(out_stream, "rc_value: %lu\n", rc_value_);
}
