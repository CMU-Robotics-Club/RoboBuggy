#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "uart.h"


#define F_CPU 16000000UL
#define BAUD 9600

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define DEBUG_DDR  DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PINN PB7 // arduino 13
#define SERVO_DDR  DDRB
#define SERVO_PORT PORTB
#define SERVO_PINN PB5 // arduino 11
#define POT_ADC_CHANNEL 0

#define SERVO_REFRESH_US 20000L
#define SERVO_MID_US 1500
#define SERVO_MIN_US 544
#define SERVO_MAX_US 2400
#define SERVO_TIMER_PRESCALER 8


static int16_t map(int32_t x,
                   int32_t in_min,
                   int32_t in_max,
                   int32_t out_min,
                   int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void servo_init(void) {
  // set up timers for PWM on OC1A (PB1, arduino pin 9)
  // prescaler of 8
  // PWM with top as ICR1 (to set cap)
  // set ICR1 to the desired refresh time
  TCCR1A |= (_BV(WGM11) | _BV(COM1A1));
  TCCR1A &= ~(_BV(WGM10) | _BV(COM1A0));
  TCCR1B |= (_BV(WGM13) | _BV(WGM12) | _BV(CS11));
  TCCR1B &= ~(_BV(CS10) | _BV(CS12));
  TCNT1 = 0;
  OCR1A = SERVO_MID_US;
  OCR1B = SERVO_MID_US;
  ICR1 = microsecondsToClockCycles(SERVO_REFRESH_US) / SERVO_TIMER_PRESCALER;

  // enable servo output
  SERVO_PORT &= ~_BV(SERVO_PINN);
  SERVO_DDR |= _BV(SERVO_PINN);
}


void servo_set_us(uint16_t value) {
  // check limits
  if(value < SERVO_MIN_US) {
    value = SERVO_MIN_US;
  }
  if(value > SERVO_MAX_US) {
    value = SERVO_MAX_US;
  }

  // modify hardware output
  OCR1A = microsecondsToClockCycles(value) / SERVO_TIMER_PRESCALER;
}


void adc_init(void) {
  // set up adc hardware in non-freerunning mode
  // prescaler of 128 gives 125kHz sampling
  // AREF = AVCC
  // 8 bit return values
  ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  ADMUX |= _BV(REFS0) | _BV(ADLAR);
}


uint8_t acd_read_blocking(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= _BV(ADSC);
  while((ADCSRA & _BV(ADSC)) == 1) {}
  return ADCH;
}


int main(void) {
  // turn the ledPin on
  DEBUG_PORT |= _BV(DEBUG_PINN);
  DEBUG_DDR |= _BV(DEBUG_PINN);
  
  // setup uart
  uart_init();
  stdout = &uart_output;
  stdin  = &uart_input;

  // enable global interrupts
  sei();

  // start servo
  servo_init();

  // setup adc
  adc_init();

  // loop forever
  while(1) {
    uint8_t pot_value = acd_read_blocking(POT_ADC_CHANNEL);
    uint16_t servo_setpoint = map(pot_value, 0, 255, SERVO_MIN_US, SERVO_MAX_US);
    printf("Read %d. Setting servo to %d.\n", pot_value, servo_setpoint);
    servo_set_us(servo_setpoint);
  }

  return 0;
}
