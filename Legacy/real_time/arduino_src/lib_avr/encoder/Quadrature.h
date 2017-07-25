#ifndef _LIB_AVR_QUAD_H_
#define _LIB_AVR_QUAD_H_

#include "Encoder.h"

class Quadrature : public Encoder {

    private:
        uint8_t config_;
        volatile uint8_t *pin_a_reg_;
        uint8_t pin_a_num_;
        volatile uint8_t *pin_b_reg_;
        uint8_t pin_b_num_;
        volatile uint8_t pin_state_last_;

    public:
        //Note: the super class's constructor is also called
        Quadrature();

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
        void OnInterrupt() override;
        /** @brief reset ticks and error counters */
        //TODO: Maybe also reset the state of the quad
        void Reset();
        void PrintDebugInfo(FILE *out_stream);
};

#endif /* _LIB_AVR_QUAD_H_ */

