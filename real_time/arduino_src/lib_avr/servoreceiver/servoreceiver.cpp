#include "servoreceiver.h"


ServoReceiver::ServoReceiver(){

}


void ServoReceiver::Init(volatile uint8_t *pin_reg, uint8_t pin_num) {
    // save instance information
    receiver_pin_reg_ = pin_reg;
    receiver_pin_num_ = pin_num;

    // set constants
    k_min_pulse_ = 500;  // min pulse width in us
    k_max_pulse_ = 2500; // max pulse width in us
    up_switch_time_ = 0; // us
    rc_value_ = 0;       // last recorded pulse width in us
    last_timestamp_ = 0; // us

    // enable interrupt handler for both edges
    EIMSK |= _BV(INT4);
    EICRB |= _BV(ISC40);
    EICRB &= ~_BV(ISC41);
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
unsigned long ServoReceiver::LastTimestamp() {
    return last_timestamp_;
}


int ServoReceiver::GetAngle(){
    int ret_val = (int)(rc_value_ - 980)*3/17; //WHYYYYY
    return ret_val;
}


void ServoReceiver::PrintDebugInfo(FILE *out_stream){
    fprintf(out_stream, "up_switch_time: %lu\n", up_switch_time_);
    fprintf(out_stream, "last_timestamp: %lu\n", last_timestamp_);
    fprintf(out_stream, "rc_value: %lu\n", rc_value_);
}
