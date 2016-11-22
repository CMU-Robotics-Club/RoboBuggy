#ifndef _LIB_AVR_RADIORECEIVER_H_
#define _LIB_AVR_RADIORECEIVER_H_

#include <avr/io.h>
#include <stdio.h>
#include "../../radio_buggy_mega/system_clock.h"


class RadioReceiver {

    private:
        volatile uint8_t *receiver_pin_reg_;
        uint8_t receiver_pin_num_;
        uint8_t int_num_;
        uint32_t k_min_pulse_;
        uint32_t k_max_pulse_;
        volatile unsigned long up_switch_time_;
        volatile unsigned long rc_value_;
        volatile unsigned long last_timestamp_;

    public:
        RadioReceiver();
        virtual void Init(volatile uint8_t *pin_reg, uint8_t pin_num, uint8_t int_num);
        void OnInterruptReceiver();
        unsigned long GetLastTimestamp();
        unsigned long GetPulseWidth();
        void PrintDebugInfo(FILE *out_stream);
};

#endif /* _LIB_AVR_RADIORECEIVER_H_ */

