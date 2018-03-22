#include "Rotary.h"

#define ENCODER_DDR  DDRD
#define ENCODER_PORT PORTD
#define ENCODER_PINN PD2 // arduino 19

Rotary::Rotary() {}

void Rotary::Init() {
    ENCODER_PORT |= _BV(ENCODER_PINN);
    ENCODER_DDR &= ~_BV(ENCODER_PINN);
    EIMSK |= _BV(INT2);
    EICRA |= _BV(ISC20);
    EICRA &= ~_BV(ISC21);
}

void Rotary::OnInterrupt()
{
    ticks_++;
}
