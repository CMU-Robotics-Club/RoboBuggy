#ifndef _LIB_AVR_ENCODER_H_
#define _LIB_AVR_ENCODER_H_

#include <avr/io.h>
#include <stdio.h>
#include "../../radio_buggy_mega/system_clock.h"

// bit locations... should be used with _BV()
/** @brief use pullups on encoder input(s) (not implemented) */
#define ENCODER_CONFIG_PULLUPS 2
/** @brief error during initialization (pins or interrupt setup) */
#define ENCODER_ERROR_INIT 0
/** @brief encoder logic reached unexpected state likely by missing an edge */
#define ENCODER_ERROR_MISSED_TICK 1
/** @brief encoder logic was called when not needed */
#define ENCODER_ERROR_EXTRA_TICK 2

/** @brief allow quadrature decoding with a single input by defining
 *
 *  Several quadrature encoder states could be reached by missing ticks (if
 *  interrupts are enabled on both edges) or by normal operation if interrupts
 *  only come from one edge. We assume one edge interrupts in this case. Some
 *  encoders that need debouncing work better by not counting these state. You
 *  should comment this define in that case.
 *
 *  Note that single int quadrature decoding did not work well in testing, but
 *  it is included in the same form in other libraries.
 */
#define ENCODER_ALLOW_SINGLE_INT

extern "C" void __cxa_pure_virtual();

class Encoder {
    protected:
        volatile uint8_t errors_;
        volatile uint8_t *pin_a_reg_;
        uint8_t pin_a_num_;
        volatile long ticks_;

    public:
        Encoder();

        /** @brief to be called on interrupt of single wire encoder signal */
        virtual void OnInterrupt() = 0;
        /** @brief get ticks seen since last reset. disables interrupts to read */
        long GetTicks();
        /** @brief get errors seen since last reset. disables interrupts to read */
        uint8_t GetErrors();
        /** @brief reset ticks and error counters */
        void Reset();
};

#endif /* _LIB_AVR_ENCODER_H_ */

