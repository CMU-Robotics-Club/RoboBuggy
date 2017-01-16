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

class Encoder {
    uint8_t config_;
    volatile uint8_t *pin_a_reg_;
    uint8_t pin_a_num_;
    volatile uint8_t *pin_b_reg_;
    uint8_t pin_b_num_;
    volatile long ticks_;
    volatile uint8_t pin_state_last_;
    volatile uint8_t errors_;

    public:
        Encoder();

        /** @brief initialize a single wire encoder */
        uint8_t Init(volatile uint8_t *pin_a_reg,
                     uint8_t pin_a_num,
                     volatile uint8_t *port_addr,
                     volatile uint8_t *ddr_addr);
        /** @brief initialize a quadrature encoder with 1 or 2 interrupts */
        uint8_t InitQuad(volatile uint8_t *pin_a_reg,
                         uint8_t pin_a_num,
                         volatile uint8_t *pin_b_reg,
                         uint8_t pin_b_num,
                         volatile uint8_t *port_a_reg,
                         volatile uint8_t *ddr_a_reg,
                         volatile uint8_t *port_b_reg,
                         volatile uint8_t *ddr_b_reg);
        /** @brief to be called on interrupt of single wire encoder signal */
        void OnInterrupt();
        /** @brief to be called by 1 or both interrupts of quadrature encoder */
        void OnInterruptQuad();
        /** @brief get ticks seen since last reset. disables interrupts to read */
        long GetTicks();
        /** @brief get errors seen since last reset. disables interrupts to read */
        uint8_t GetErrors();
        /** @brief reset ticks and error counters */
        void Reset();
        void PrintDebugInfo(FILE *out_stream);
};

#endif /* _LIB_AVR_ENCODER_H_ */

