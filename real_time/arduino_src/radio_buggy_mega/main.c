#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


#define DEBUG_DDR  DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PIN  PB5 // arduino 13

#define LED_FRAME_COUNT 15
#define LED_FRAME_DELAY 1000



int main(void) {
  // set up timers for PWM on OC1A (PB1, arduino pin 9)
  TCCR1A = 0;             // normal counting mode 
  TCCR1B = _BV(CS11);     // set prescaler of 8 
  TCNT1 = 0;              // clear the timer count
  TIFR1 |= _BV(OCF1A);      // clear any pending interrupts; 
  TIMSK1 |=  _BV(OCIE1A) ;  // enable the output compare interrupt 
  
  // turn the ledPin on
  // digitalWrite(led_pin, HIGH);
  DEBUG_PORT |= _BV(DEBUG_PIN);
  DEBUG_DDR |= _BV(DEBUG_PIN);
  
  // Serial.begin(9600);

  // loop forever
  while(1) {
    for (int i = 544; i <= 2400; i++) {
      OCR1A = i / 4;
      
      // Serial.println("hi");
      _delay_ms(1000);
    }
  }

  return 0;
}
