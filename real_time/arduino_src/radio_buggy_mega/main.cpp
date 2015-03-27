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
#define CONNECTION_TIMEOUT_US 1000000L // 1000ms

#define PWM_OFFSET_STEERING_OUT 1789
#define PWM_SCALE_STEERING_OUT -150
#define PWM_OFFSET_STORED_ANGLE 0
#define PWM_SCALE_STORED_ANGLE 1000 // in hundredths of a degree for precision

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

#define LED_DANGER_DDR  DDRB
#define LED_DANGER_PORT PORTB
#define LED_DANGER_PINN PB6 // arduino 12
#define BRAKE_OUT_DDR  DDRH
#define BRAKE_OUT_PORT PORTH
#define BRAKE_OUT_PINN PH5 // arduino 8
#define BRAKE_INDICATOR_DDR  DDRE
#define BRAKE_INDICATOR_PORT PORTE
#define BRAKE_INDICATOR_PINN PE3 // arduino 5

// Global state
static bool g_brake_state_engaged; // 0 = disengaged, !0 = engaged.
static bool g_brake_needs_reset; // 0 = nominal, !0 = needs reset
static bool g_is_autonomous;
static unsigned long g_current_voltage;
static int smoothed_thr;
static int smoothed_auton;
static int steer_angle;
static int auto_steering_angle;

RBSerialMessages g_rbsm_endpoint;
rb_message_t g_new_rbsm;
ServoReceiver g_steering_rx;
ServoReceiver g_brake_rx;
ServoReceiver g_auton_rx;


inline long map_signal(long x,
                       long in_offset,
                       long in_scale,
                       long out_offset,
                       long out_scale) {
  return ((x - in_offset) * out_scale / in_scale) + out_offset;
}


void adc_init(void) {
  // set up adc hardware in non-freerunning mode
  // prescaler of 128 gives 125kHz sampling
  // AREF = AVCC
  // 8 bit return values
  ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  ADMUX |= _BV(REFS0) | _BV(ADLAR);
}


uint8_t adc_read_blocking(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= _BV(ADSC);
  while((ADCSRA & _BV(ADSC)) == 1) {}
  return ADCH;
}


void steering_set(int angle) {
  int servo_value_us = map_signal(angle,
                                  PWM_OFFSET_STORED_ANGLE,
                                  PWM_SCALE_STORED_ANGLE,
                                  PWM_OFFSET_STEERING_OUT,
                                  PWM_SCALE_STEERING_OUT);
  servo_set_us(servo_value_us);
}


void brake_init() {
  BRAKE_OUT_DDR |= _BV(BRAKE_OUT_PINN);
  BRAKE_INDICATOR_DDR |= _BV(BRAKE_INDICATOR_PINN);
}


// Note: High-voltage raises the brake.
// Raises the brake
// Do not call before brake_init
void brake_raise() {
  BRAKE_OUT_PORT |= _BV(BRAKE_OUT_PINN);
  BRAKE_INDICATOR_PORT &= ~_BV(BRAKE_INDICATOR_PINN);
}


// Drops the brake
// Do not call before brake_init
void brake_drop() {
  BRAKE_OUT_PORT &= ~_BV(BRAKE_OUT_PINN);
  BRAKE_INDICATOR_PORT |= _BV(BRAKE_INDICATOR_PINN);
}


int main(void) {
  // turn the ledPin on
  // DEBUG_PORT |= _BV(DEBUG_PINN);
  DEBUG_PORT &= ~_BV(DEBUG_PINN);
  DEBUG_DDR |= _BV(DEBUG_PINN);

  // setup hardware
  sei(); // enable interrupts
  uart_init();
  system_clock_init();
  servo_init();
  adc_init();
  brake_init();
  LED_DANGER_DDR |= _BV(LED_DANGER_PINN);
  
  // setup rbsm
  g_rbsm_endpoint.Init(&uart_stdio, &uart_stdio);

  // set up rc receivers
  g_steering_rx.Init(&RX_STEERING_PIN, RX_STEERING_PINN, RX_STEERING_INTN);
  g_brake_rx.Init(&RX_BRAKE_PIN, RX_BRAKE_PINN, RX_BRAKE_INTN);
  g_auton_rx.Init(&RX_AUTON_PIN, RX_AUTON_PINN, RX_AUTON_INTN);

  // loop forever
  while(1) {
    // get new command messages
    // TODO: make reading messages work
    //
    // rb_message_t new_command;
    // int read_status;
    
    // while((read_status = g_rbserialmessages.Read(&new_command))
    //       != RBSM_ERROR_INSUFFICIENT_DATA) {
    //   if(read_status == false) {
    //     // dipatch complete message
    //     switch(new_command.message_id) {
    //       case RBSM_MID_MEGA_STEER_ANGLE:
    //         auto_steering_angle = (int)(long)new_command.data;
    //         break;

    //       default:
    //         // report unknown message
    //         g_rbserialmessages.Send(RBSM_MID_ERROR, RBSM_EID_RBSM_INVALID_MID);
    //         dbg_println("Got message with invalid mid:");
    //         dbg_println(new_command.message_id);
    //         dbg_println(new_command.data);
    //         break;
    //     }
    //   } else if(read_status == RBSM_ERROR_INVALID_MESSAGE) {
    //     // report stream losses for tracking
    //     g_rbserialmessages.Send(RBSM_MID_ERROR, RBSM_EID_RBSM_LOST_STREAM);
    //   }
    //   // drop responses with other faults
    // }

    // find the new steering angle, if available
    steer_angle = g_steering_rx.GetAngleThousandths();

    // find the new brake state, if available
    smoothed_thr = g_brake_rx.GetAngle();
    // TODO make this code...less...something
    if(smoothed_thr > 120) {
      // read as engaged
      g_brake_state_engaged = true;
      // brake has been reset
      g_brake_needs_reset = false;
    } else {
      // read as disengaged
      g_brake_state_engaged = false;
    }

    // find the new autonomous state, if available
    smoothed_auton = g_auton_rx.GetAngle();
    // TODO make this code...less...something
    if(smoothed_auton > 120) { // MAGIC NUMBERS
      // read as engaged
      g_is_autonomous = true;
    } else {
      // read as disengaged
      g_is_autonomous = false;
    }

    // detect dropped conections
    // note: interrupts must be disabled while checking system clock so that
    //       timestamps are not updated under our feet
    cli();
    unsigned long time_now = micros();
    unsigned long time1 = g_steering_rx.GetLastTimestamp();
    unsigned long time2 = g_brake_rx.GetLastTimestamp();
    unsigned long time3 = g_auton_rx.GetLastTimestamp();
    unsigned long delta1 = time_now - time1;
    unsigned long delta2 = time_now - time2;
    unsigned long delta3 = time_now - time3;
    sei();
    if(delta1 > CONNECTION_TIMEOUT_US ||
       delta2 > CONNECTION_TIMEOUT_US ||
       delta3 > CONNECTION_TIMEOUT_US) {
      // we haven't heard from the RC receiver in too long
      if(g_brake_needs_reset == false) {
        g_rbsm_endpoint.Send(RBSM_MID_ERROR, RBSM_EID_RC_LOST_SIGNAL);
      }
      g_brake_needs_reset = true;
    }


    //For the old buggy, the voltage divider is 10k ohm on the adc side and 16k ohm on top.
    g_current_voltage = map_signal(adc_read_blocking(0), 0, 255, 0, 12636); //in millivolts
    //normally set to 13000, but the avcc is 4.86 volts, rather than 5.
    // Set outputs
    if(g_brake_state_engaged == false && g_brake_needs_reset == false) {
      brake_raise();
    } else {
      brake_drop();
    }

    if(g_is_autonomous){
      steering_set(auto_steering_angle);
      g_rbsm_endpoint.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)(auto_steering_angle));
    }
    else if(!g_is_autonomous){
      steering_set(steer_angle);
      g_rbsm_endpoint.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)steer_angle);
    }
    else{
      // dbg_println("Somehow not in either autonomous or teleop");
      steering_set(PWM_OFFSET_STORED_ANGLE); // default to centered
    }

    if(g_brake_needs_reset == true) {
      LED_DANGER_PORT |= _BV(LED_DANGER_PINN);
    } else {
      LED_DANGER_PORT &= ~_BV(LED_DANGER_PINN);
    }

    // Send telemetry messages
    g_rbsm_endpoint.Send(RBSM_MID_DEVICE_ID, RBSM_DID_DRIVE_ENCODER);
    // g_rbsm_endpoint.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)steer_angle);
    g_rbsm_endpoint.Send(RBSM_MID_MEGA_BRAKE_STATE, 
                            (long unsigned)g_brake_state_engaged);
    g_rbsm_endpoint.Send(RBSM_MID_MEGA_AUTON_STATE,
                            (long unsigned)g_is_autonomous);
    g_rbsm_endpoint.Send(RBSM_MID_MEGA_BATTERY_LEVEL, g_current_voltage);

  }


  return 0;
}


ISR(RX_STEERING_INT) {
  g_steering_rx.OnInterruptReceiver();
}


ISR(RX_BRAKE_INT) {
  g_brake_rx.OnInterruptReceiver();
}


ISR(RX_AUTON_INT) {
  g_auton_rx.OnInterruptReceiver();
}
