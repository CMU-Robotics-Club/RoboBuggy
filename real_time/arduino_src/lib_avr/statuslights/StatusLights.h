#ifndef _LIB_AVR_STATUSLIGHTS_H_
#define _LIB_AVR_STATUSLIGHTS_H_

#include <avr/io.h>
#include <stdio.h>
#include "../../radio_buggy_mega/system_clock.h"


class StatusLights {
    protected:
        volatile uint8_t *light_port;
        uint8_t register_offset;

    public:
        StatusLights(uint8_t reg_offset, volatile uint8_t *port, volatile uint8_t *ddr);
        void Enable(void);
        void Disable(void);
        void Flash(uint8_t time);
        void FlashBlocking(uint8_t time);
};

#endif /* _LIB_AVR_STATUSLIGHTS_H_ */

