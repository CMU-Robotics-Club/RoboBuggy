#include "Encoder.h"

extern "C" void __cxa_pure_virtual() { while (1); }

Encoder::Encoder() 
{
    ticks_ = 0;
    errors_ = 0;
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

