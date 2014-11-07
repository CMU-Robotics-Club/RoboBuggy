#include "pinreceiver.h"
#include <Arduino.h>

PinReceiver::PinReceiver(){

}

void PinReceiver::Begin(int pin_num, int int_num, void (*int_wrapper)() ){
    receiver_pin_ = pin_num;
    k_pwm_time_ = 18370;
    k_pwm_thresh_ = 1300;
    k_big_pulse_ = 8;
    k_short_pulse_ = 19;
    rc_available_ = 0;
    up_switch_time_ = 0;
    down_switch_time_ = 0;
    rc_value_ = 0;

    attachInterrupt(int_num, int_wrapper, CHANGE);
}


void PinReceiver::OnInterruptReceiver(){
    if (digitalRead(receiver_pin_) == HIGH){
        if ((micros() - up_switch_time_ > k_pwm_time_ - k_pwm_thresh_) &&
            (micros() - up_switch_time_ < k_pwm_time_ + k_pwm_thresh_)) {
            if ((down_switch_time_ > up_switch_time_) && (rc_available_ == 0)) {
                rc_value_ = down_switch_time_ - up_switch_time_;
                rc_available_ = 1;
                if (rc_value_ / (k_pwm_time_ / k_big_pulse_) >= 1 ||
                    rc_value_ / (k_pwm_time_ / k_short_pulse_) == 0) {
                    rc_available_ == 0;
                }
            }
        }
        up_switch_time_ = micros();
    } else {
        if (up_switch_time_) {
            down_switch_time_ = micros();

        }
    }
}


bool PinReceiver::Available() {
    return rc_available_;
}


int PinReceiver::GetAngle(){
    int ret_val = (int)(rc_value_ - 980)*3/17; //WHYYYYY
    rc_available_ = 0;
    return ret_val;
}


void PinReceiver::PrintDebugInfo(){
    Serial1.print("up_switch_time: ");
    Serial1.println(up_switch_time_);
    Serial1.print("down_switch_time: ");
    Serial1.println(down_switch_time_);
    Serial1.print("rc_available: ");
    Serial1.println(rc_available_);
    Serial1.print("rc_value: ");
    Serial1.println(rc_value_);
}
