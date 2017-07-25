#include "Encoder.h"


Encoder::Encoder() 
{
    ticks_ = 0;
    errors_ = 0;
}

uint8_t Encoder::Init(volatile uint8_t *pin_a_reg,
                      uint8_t pin_a_num) 
{
    // save configuration info
    pin_a_reg_ = pin_a_reg;
    pin_a_num_ = pin_a_num;

    return errors_;
}

long Encoder::GetTicks() 
{
    cli();
    long ticks_to_return = ticks_;
    sei();
    return ticks_to_return;
}


uint8_t Encoder::GetErrors() 
{
    cli();
    uint8_t errors_to_return = errors_;
    sei();
    return errors_to_return;
}


void Encoder::Reset() 
{
    cli();
    ticks_ = 0;
    errors_ = 0;
    sei();
}

void Encoder::OnInterrupt()
{
    cli();
    ticks_++;
    sei();
}
