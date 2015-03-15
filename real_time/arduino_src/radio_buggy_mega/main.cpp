#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "../lib_avr/rbserialmessages/rbserialmessages.h"
#include "../lib_avr/servoreceiver/servoreceiver.h"
#include "servo.h"
#include "uart.h"
#include "system_clock.h"


#define BAUD 9600
#define DEBUG_DDR  DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PINN PB7 // arduino 13
#define SERVO_DDR  DDRB
#define SERVO_PORT PORTB
#define SERVO_PINN PB5 // arduino 11 TODO: this is not used here
#define POT_ADC_CHANNEL 0

#define RX_STEERING_DDR  DDRE
#define RX_STEERING_PORT PORTE
#define RX_STEERING_PIN  PINE
#define RX_STEERING_PINN PE4 // arduino 2
#define RX_STEERING_INT  INT4_vect
#define RX_STEERING_INTN 4
#define RX_BRAKE_DDR  DDRD
#define RX_BRAKE_PORT PORTD
#define RX_BRAKE_PIN  PIND
#define RX_BRAKE_PINN PD0 // arduino 21
#define RX_BRAKE_INT  INT0_vect
#define RX_BRAKE_INTN 0
#define RX_AUTON_DDR  DDRD
#define RX_AUTON_PORT PORTD
#define RX_AUTON_PIN  PIND
#define RX_AUTON_PINN PD1 // arduino 20
#define RX_AUTON_INT  INT1_vect
#define RX_AUTON_INTN 1

RBSerialMessages g_rbsm_endpoint;
rb_message_t g_new_rbsm;
ServoReceiver g_steering_rx;
ServoReceiver g_brake_rx;
ServoReceiver g_auton_rx;

static int raw_angle;
static int smoothed_angle;



static int16_t map(int32_t x,
                   int32_t in_min,
                   int32_t in_max,
                   int32_t out_min,
                   int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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


int convert_rc_to_steering(int rc_angle) {
  //Inverter for 2.4 GHz racecar received
  rc_angle = 180-rc_angle;
  int out = (rc_angle/4)+(90*3/4)+36;
  if(out < 100 || out > 155) {
    // dbg_println("FAKFAKFAK SERVO OUT OF RANGE");
    // dbg_println(out);
    out = 125;
  }
  return out;
}


int main(void) {
  // turn the ledPin on
  // DEBUG_PORT |= _BV(DEBUG_PINN);
  DEBUG_PORT &= ~_BV(DEBUG_PINN);
  DEBUG_DDR |= _BV(DEBUG_PINN);
  
  // setup rbsm
  uart_init();
  g_rbsm_endpoint.Init(&uart_stdio, &uart_stdio);

  // enable global interrupts
  sei();

  // start servo
  servo_init();

  // setup adc
  adc_init();

  //set up system clock
  system_clock_init();

  // set up rc receiver
  g_steering_rx.Init(&RX_STEERING_PIN, RX_STEERING_PINN, RX_STEERING_INTN);

  // loop forever
  while(1) {
    uint8_t pot_value = acd_read_blocking(POT_ADC_CHANNEL);
    uint16_t servo_setpoint = map(pot_value, 0, 255, SERVO_MIN_US, SERVO_MAX_US);
    servo_set_us(servo_setpoint);

    // find the new steering angle, if available
    raw_angle = g_steering_rx.GetAngle();
    smoothed_angle = convert_rc_to_steering(raw_angle);


    // test sending serial messages
    g_rbsm_endpoint.Send(RBSM_MID_MEGA_STEER_ANGLE, smoothed_angle);

    // try to process new messages
    // TODO: update UART code to include buffering needed for this to work
    //
    // DEBUG_PORT &= ~_BV(DEBUG_PINN);
    // switch(g_rbsm_endpoint.Read(&g_new_rbsm)) {

    //   case 0:
    //     // send this id back
    //     g_rbsm_endpoint.Send(RBSM_MID_ERROR, g_new_rbsm.data);
    //     break;

    //   default:
    //     // just ignore it
    //     break;
    // }
    // DEBUG_PORT |= _BV(DEBUG_PINN);
  }


  return 0;
}


ISR(RX_STEERING_INT) {
  g_steering_rx.OnInterruptReceiver();
}


// ISR(RX_BRAKE_INT) {
//   g_brake_rx.OnInterruptReceiver();
// }


// ISR(RX_AUTON_INT) {
//   g_auton_rx.OnInterruptReceiver();
// }
