#ifndef _LIB_AVR_SERVORECEIVER_H_
#define _LIB_AVR_SERVORECEIVER_H_

#include <avr/io.h>
#include <stdio.h>
#include "../../radio_buggy_mega/system_clock.h"

class ServoReceiver {
    volatile uint8_t *receiver_pin_reg_;
    uint8_t receiver_pin_num_;
    int k_min_pulse_;
    int k_max_pulse_;
    volatile unsigned long up_switch_time_;
    volatile unsigned long rc_value_;
    unsigned long last_timestamp_;

    public:
        ServoReceiver();
        void Init(volatile uint8_t *pin_reg, uint8_t pin_num);
        void OnInterruptReceiver();
        unsigned long LastTimestamp();
        int GetAngle();
        void PrintDebugInfo(FILE *out_stream);
};

#endif /* _LIB_AVR_SERVORECEIVER_H_ */

