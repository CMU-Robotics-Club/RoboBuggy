#include "Rotary.h"


Rotary::Rotary() {}

uint8_t Rotary::Init(volatile uint8_t *pin_a_reg,
                      uint8_t pin_a_num,
                      volatile uint8_t *port_reg,
                      volatile uint8_t *ddr_reg) {
    // save configuration info
    pin_a_reg_ = pin_a_reg;
    pin_a_num_ = pin_a_num;

    // reset state
    ticks_ = 0;
    errors_ = 0;

    // setup encoder pin with pullups and interrupt
    *port_reg |= _BV(pin_a_num);
    *ddr_reg &= ~_BV(pin_a_num);

    //TODO: Too hard coded.  
    //Maybe turn into input arguments? However, arg list is getting long...
    //External interrupt mask register. Enables external interrupt on pin PD2
    EIMSK |= _BV(INT2);
    //External interrupt control register A.  Configures interrupt to be triggered on any edge
    EICRA |= _BV(ISC20);
    EICRA &= ~_BV(ISC21);

    return errors_;
}

void Rotary::OnInterrupt()
{
    cli();
    ticks_++;
    sei();
}
