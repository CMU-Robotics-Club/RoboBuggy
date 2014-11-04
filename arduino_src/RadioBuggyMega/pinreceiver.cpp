#include "pinreceiver.h"
#include <Arduino.h>

PinReceiver::PinReceiver(){

}

void PinReceiver::Begin(int pin, int int_num, void (*int_wrapper)() ){
    receiver_pin_ = pin;
    switch(receiver_pin_){
        case 2:
            leftmost = 2000;
            rightmost = 980;
            centermost = 1480;
        case 3:
            leftmost = -1;
            rightmost = -1;
            centermost = -1;
        default:
            leftmost = 0;
            rightmost = 0;
            centermost = 0;
    }
    pwm_time = 18370;
    pwm_thresh = 1300;
    big_pulse = 8;
    short_pulse = 19;
    // signal_time = 0;
    // signal_diff_time = 0;
    rc_available = 0;
    up_switch_time = 0;
    down_switch_time = 0;
    rc_value = 0;

    attachInterrupt(int_num, int_wrapper, CHANGE);
}


void PinReceiver::OnInterruptReceiver(){
    if (digitalRead(receiver_pin_) == HIGH){
        if ((micros() - up_switch_time > pwm_time - pwm_thresh) &&
            (micros() - up_switch_time < pwm_time + pwm_thresh)) {
            if ((down_switch_time > up_switch_time) && (rc_available == 0)) {
                rc_value = down_switch_time - up_switch_time;
                rc_available = 1;
                if (rc_value / (pwm_time / big_pulse) >= 1 ||
                    rc_value / (pwm_time / short_pulse) == 0) {
                    rc_available == 0;
                }
            }
        }
        up_switch_time = micros();
    } else {
        if (up_switch_time) {
            down_switch_time = micros();

        }
    }
}


bool PinReceiver::Available() {
    return rc_available;
}


int PinReceiver::GetAngle(){
    int ret_val = (int)(rc_value - 980)*(3/17); //WHYYYYY
    rc_available = 0;
    return ret_val;
}