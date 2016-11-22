#ifndef _LIB_AVR_SERVORECEIVER_H_
#define _LIB_AVR_SERVORECEIVER_H_

#include <avr/io.h>
#include <stdio.h>
#include "../../radio_buggy_mega/system_clock.h"
#include "RadioReceiver.h"


class ServoReceiver : public RadioReceiver {
    private: 
        volatile uint8_t *receiver_pin_reg_;
        uint8_t receiver_pin_num_;
        uint32_t k_min_pulse_;
        uint32_t k_max_pulse_;
        int k_offset_rc_in;
        int k_scale_rc_in;
        int k_offset_steering_out;
        int k_offset_stored_angle;
        int k_scale_stored_angle;
        int k_scale_steering_out;
        volatile unsigned long up_switch_time_;
        volatile unsigned long rc_value_;
        volatile unsigned long last_timestamp_;

    public:
        ServoReceiver();
        void Init(volatile uint8_t *pin_reg, uint8_t pin_num, uint8_t int_num);
        int GetAngleHundredths();
};

#endif /* _LIB_AVR_SERVORECEIVER_H_ */

