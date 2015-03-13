#ifndef LIB_AVR_SYSTEM_CLOCK_H
#define LIB_AVR_SYSTEM_CLOCK_H

#include <avr/io.h>
#include <avr/interrupt.h>

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void system_clock_init(void);
void delay_microseconds(unsigned int us);
void delay(unsigned long ms);
unsigned long micros(void);
unsigned long millis(void);

#endif
