/*
* main.cpp
*
* This file contains the main body of code for the low-level systems. This 
*  includes interactions with the high-level, brake, and steering systems.
*
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/wdt.h>

#include "../lib_avr/rbserialmessages/rbserialmessages.h"
#include "../lib_avr/servoreceiver/servoreceiver.h"
#include "../lib_avr/uart/uart_extra.h"
#include "servo.h"
// #include "uart.h"
#include "system_clock.h"
#include "fingerprint.h"

// Use 76800 or 250k. 115200 does not work well with 16MHz clock.
// ATMega 2560 datasheet Table 22-12.
#define BAUD 76800

#define DEBUG_DDR  DDRB
#define DEBUG_PORT PORTB
#define DEBUG_PINN PB7 // arduino 13
#define SERVO_DDR  DDRB
#define SERVO_PORT PORTB
#define SERVO_PINN PB5 // arduino 11 TODO: this is not used here
#define CONNECTION_TIMEOUT_US 1000000L // 1000ms

/*
 * These values map the physical input/output (voltage/ms of pwm pulse) to a
 * fixed physical angle. When software commands a steering angle of 10 deg
 * (1000 hundredths), the PWM_SCALE_STEERING_OUT and POT_SCALE_STEERING_IN
 * should be adjusted until a 10 deg input/output is seen/observed.
 */
#if BUGGY == transistor 
    #define PWM_OFFSET_STEERING_OUT 1905
    #define PWM_SCALE_STEERING_OUT -220
    #define PWM_OFFSET_STORED_ANGLE 0
    #define PWM_SCALE_STORED_ANGLE 1000 // in hundredths of a degree for precision
    #define POT_OFFSET_STEERING_IN 121
    #define POT_SCALE_STEERING_IN -10
    #define STEERING_LIMIT_LEFT -1000
    #define STEERING_LIMIT_RIGHT 1000
#elif BUGGY == nixie
    #define PWM_OFFSET_STEERING_OUT 1789
    #define PWM_SCALE_STEERING_OUT -150
    #define PWM_OFFSET_STORED_ANGLE 0
    #define PWM_SCALE_STORED_ANGLE 1000 // in hundredths of a degree for precision
#else
    #error "must compile with BUGGY_TRANSISTOR or BUGGY_NIXI flag"
#endif

#define PWM_STATE_THRESHOLD 120

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

#define ENCODER_DDR  DDRD
#define ENCODER_PORT PORTD
#define ENCODER_PIN  PIND
#define ENCODER_PINN PD2 // arduino 19
#define ENCODER_INT  INT2_vect
#define ENCODER_INTN 2
#define ENCODER_TIMEOUT_US 500 // 50mph w/ 6" wheel = 280 ticks/sec; 4000us/tick

#define BATTERY_ADC 0
#define STEERING_POT_ADC 9

#define BRAKE_OUT_DDR  DDRH
#define BRAKE_OUT_PORT PORTH
#define BRAKE_OUT_PINN PH5 // arduino 8
#define BRAKE_INDICATOR_DDR  DDRE
#define BRAKE_INDICATOR_PORT PORTE
#define BRAKE_INDICATOR_PINN PE3 // arduino 5

#define WDT_INT WDT_vect

//Uncomment the line below for testing. Otherwise leave commented out
// #define DEBUG

#ifdef DEBUG
    #define dbg_printf(...) printf(__VA_ARGS__)
#else
    #define dbg_printf 
#endif

#define min(X,Y) ((X < Y) ? (X) : (Y))
#define max(X,Y) ((X > Y) ? (X) : (Y))


// Global state
static bool g_is_autonomous;
static unsigned long g_current_voltage;
static unsigned long g_steering_feedback;
static volatile unsigned long g_encoder_ticks;
static int steer_angle;
static int auto_steering_angle;
static unsigned long g_errors;

RBSerialMessages g_rbsm;
rb_message_t g_new_rbsm;
ServoReceiver g_steering_rx;
ServoReceiver g_brake_rx;
ServoReceiver g_auton_rx;

UARTFILE g_uart_rbsm;
UARTFILE g_uart_debug;


inline long map_signal(long x,
                       long in_offset,
                       long in_scale,
                       long out_offset,
                       long out_scale) 
{
    return ((x - in_offset) * out_scale / in_scale) + out_offset;
}

inline long clamp(long input,
                  long upper,
                  long lower)
{
    return (max(min(input, upper), lower));
}


void adc_init(void) 
{
    // set up adc hardware in non-freerunning mode
    // prescaler of 128 gives 125kHz sampling
    // AREF = AVCC
    // 8 bit return values
    ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
    ADMUX |= _BV(REFS0) | _BV(ADLAR);
}

uint8_t adc_read_blocking(uint8_t channel) 
{
    // in single ended mode, highest bit of ADC channel is in ADCSRB
    ADMUX = (ADMUX & 0xE0) | (channel & 0x07);
    ADCSRB = (ADCSRB & 0xF7) | (channel & 0x08);
    ADCSRA |= _BV(ADSC);
    while((ADCSRA & _BV(ADSC)) == 1) {}
    return ADCH;
}


void steering_set(int angle) 
{
    angle = clamp(angle, 
                  STEERING_LIMIT_RIGHT,
                  STEERING_LIMIT_LEFT);

    int servo_value_us = map_signal(angle,
                                    PWM_OFFSET_STORED_ANGLE,
                                    PWM_SCALE_STORED_ANGLE,
                                    PWM_OFFSET_STEERING_OUT,
                                    PWM_SCALE_STEERING_OUT);
    servo_set_us(servo_value_us);
}


void brake_init() 
{
    BRAKE_OUT_DDR |= _BV(BRAKE_OUT_PINN);
    BRAKE_INDICATOR_DDR |= _BV(BRAKE_INDICATOR_PINN);
}


// Note: High-voltage raises the brake.
// Do not call before brake_init
void brake_raise() 
{
    BRAKE_OUT_PORT |= _BV(BRAKE_OUT_PINN);
    BRAKE_INDICATOR_PORT &= ~_BV(BRAKE_INDICATOR_PINN);
}


// Drops the brake
// Do not call before brake_init
void brake_drop() 
{
    BRAKE_OUT_PORT &= ~_BV(BRAKE_OUT_PINN);
    BRAKE_INDICATOR_PORT |= _BV(BRAKE_INDICATOR_PINN);
}

/*
* Function: watchdog_init
*
* Description: Configures the hardware watchdog to monitor for timeouts.
*   Sets the system to "bark" after 1 second without resets.  Has an 
*   independent clock and will check for timeout regardless of main code.
*/
void watchdog_init()
{
    //Disable interrupts because setup is time sensitive
    cli();

    //Set the watchdog timer control register
    wdt_reset();

    //This line enables the watchdog timer to be configured. Do not change.
    WDTCSR |= (1 << WDCE) | (1 << WDE);

    //This next line must run within 4 clock cycles of the above line.
    //See Section 12.4 of the ATMEGA2560 datasheet for additional details
    WDTCSR = 
            (1 << WDIE) //Call ISR on timeout
            | (1 << WDP2) // w/ WDP1 sets timeout to 1 second
            | (1 << WDP1);

    //Re-enable interrupts
    sei();
}


int main(void) 
{
    // state variables
    bool brake_needs_reset = true; // 0 = nominal, !0 = needs reset
    bool brake_cmd_teleop_engaged = false;
    bool brake_cmd_auton_engaged = false;
    unsigned long time_start = micros();
    unsigned long auton_brake_last = time_start;
    unsigned long auton_steer_last = time_start;

    // turn the ledPin on
    // DEBUG_PORT |= _BV(DEBUG_PINN);
    DEBUG_PORT &= ~_BV(DEBUG_PINN);
    DEBUG_DDR |= _BV(DEBUG_PINN);

    // setup encoder pin and interrupt
    ENCODER_PORT |= _BV(ENCODER_PINN);
    ENCODER_DDR &= ~_BV(ENCODER_PINN); 
    EIMSK |= _BV(INT2);
    EICRA |= _BV(ISC20) | _BV(ISC21);

    // prepare uart0 (onboard usb) for rbsm
    uart0_init(UART_BAUD_SELECT(BAUD, F_CPU));
    uart0_fdevopen(&g_uart_rbsm);

    // prepare uart2 (because servo conflicts with uart1) for debug output
    uart2_init(UART_BAUD_SELECT(BAUD, F_CPU));
    uart2_fdevopen(&g_uart_debug);
    // map stdio for printf
    stdin = stdout = stderr = &g_uart_debug;

    // setup hardware
    sei(); // enable interrupts
    system_clock_init();
    servo_init();
    adc_init();
    brake_init();

    // servo power control starts outputting low
    // PBL0 = Arduino 49
    PORTL &= ~_BV(0);
    DDRL |= _BV(0);

    // setup rbsm
    g_rbsm.Init(&g_uart_rbsm, &g_uart_rbsm);

    // set up rc receivers
    g_steering_rx.Init(&RX_STEERING_PIN, RX_STEERING_PINN, RX_STEERING_INTN);
    g_brake_rx.Init(&RX_BRAKE_PIN, RX_BRAKE_PINN, RX_BRAKE_INTN);
    g_auton_rx.Init(&RX_AUTON_PIN, RX_AUTON_PINN, RX_AUTON_INTN);

    //Output information about code on arduino once to uart2 on startup.
    printf("Hello world! This is debug information\r\n");
    printf("Compilation date: %s\r\n", FP_COMPDATE);
    printf("Compilation time: %s\r\n", FP_COMPTIME);
    printf("Branch name: %s\r\n", FP_BRANCHNAME);
    printf("Most recent commit: %s\r\n", FP_STRCOMMITHASH);
    printf("Branch clean? %d\r\n", FP_CLEANSTATUS);
    printf("\nEnd of compilation information\r\n");

    watchdog_init();

    // loop forever
    while(1) 
    {
        // enable servo power after timeout
        if(millis() > 200) 
        {
            PORTL |= _BV(0);
        }

        // Check for incomming serial messages
        rb_message_t new_command;
        int read_status;

        while((read_status = g_rbsm.Read(&new_command))
             != RBSM_ERROR_INSUFFICIENT_DATA) 
        {
            if(read_status == 0) 
            {
                // clear RBSM errors
                g_errors &= ~_BV(RBSM_EID_RBSM_LOST_STREAM);
                g_errors &= ~_BV(RBSM_EID_RBSM_INVALID_MID);
                // dipatch complete message
                switch(new_command.message_id)
                {
                    case RBSM_MID_ENC_RESET_REQUEST:
                        cli();
                        g_encoder_ticks = 0;
                        sei();
                        //Let high level know that the request went through
                        g_rbsm.Send(RBSM_MID_ENC_RESET_CONFIRM, 1);
                        dbg_printf("Encoder reset request received!\n");
                        break;
                    case RBSM_MID_MEGA_STEER_COMMAND:
                        auto_steering_angle = (int)(long)new_command.data;
                        auton_steer_last = micros();
                        dbg_printf("Got steering message for %d.\n", auto_steering_angle);
                        break;
                    case RBSM_MID_MEGA_AUTON_BRAKE_COMMAND:
                        brake_cmd_auton_engaged = (bool)(long)new_command.data;
                        auton_brake_last = micros();
                        dbg_printf("Got brake message for %d.\n", brake_cmd_auton_engaged);
                        break;
                    default:
                        // report unknown message
                        dbg_printf("Got message with invalid mid %d and data %d\n",
                             new_command.message_id,
                             new_command.data);
                        g_errors |= _BV(RBSM_EID_RBSM_INVALID_MID);
                        break;
                } //End switch(new_command.message_id)
            }
            // report stream losses for tracking
            else if(read_status == RBSM_ERROR_INVALID_MESSAGE) 
            {
                dbg_printf("RBSM could not parse message.\n");
                g_errors |= _BV(RBSM_EID_RBSM_LOST_STREAM);
            }
            // should not be other faults
            else {
                dbg_printf("Unknown RBSM parse error.\n");
                g_errors |= _BV(RBSM_EID_RBSM_LOST_STREAM);
            }
        }

        // Check for RC commands
        steer_angle = g_steering_rx.GetAngleHundredths();
        brake_cmd_teleop_engaged = g_brake_rx.GetAngle() > PWM_STATE_THRESHOLD;
        g_is_autonomous = g_auton_rx.GetAngle() > PWM_STATE_THRESHOLD;

        // Detect dropped radio conections
        // note: interrupts must be disabled while checking system clock so that
        //       timestamps are not updated under our feet
        cli(); //disable interrupts
        unsigned long time_now = micros();
        unsigned long time1 = g_steering_rx.GetLastTimestamp();
        unsigned long time2 = g_brake_rx.GetLastTimestamp();
        unsigned long time3 = g_auton_rx.GetLastTimestamp();
        //RC time deltas
        unsigned long delta1 = time_now - time1;
        unsigned long delta2 = time_now - time2;
        unsigned long delta3 = time_now - time3;
        //Auton time deltas
        unsigned long delta5 = time_now - auton_steer_last;
        unsigned long delta4 = time_now - auton_brake_last;

        unsigned long g_encoder_ticks_safe = g_encoder_ticks;
        sei(); //enable interrupts

        bool rc_timeout = delta1 > CONNECTION_TIMEOUT_US ||
                          delta2 > CONNECTION_TIMEOUT_US ||
                          delta3 > CONNECTION_TIMEOUT_US;
        bool auton_timeout = (g_is_autonomous == true) &&
                             (delta4 > CONNECTION_TIMEOUT_US ||
                              delta5 > CONNECTION_TIMEOUT_US);
        if(rc_timeout || auton_timeout) {
            // check for RC timout first
            if(rc_timeout) {
                // we haven't heard from the RC receiver in too long
                dbg_printf("Timed out connection from RC!\n");
                g_errors |= _BV(RBSM_EID_RC_LOST_SIGNAL);
                brake_needs_reset = true;
            }
            else {
                g_errors &= ~_BV(RBSM_EID_RC_LOST_SIGNAL);
            }

            // then check for timeout under autonomous
            if(auton_timeout) {
                dbg_printf("Timed out connection from high level!\n");
                g_errors |= _BV(RBSM_EID_AUTON_LOST_SIGNAL);
                brake_needs_reset = true;
            }
            else {
                g_errors &= ~_BV(RBSM_EID_AUTON_LOST_SIGNAL);
            }
        }

        // or reset the system if connection is back and driver engages brakes
        else {
            if(brake_cmd_teleop_engaged == true) {
                brake_needs_reset = false;
            }

            g_errors &= ~_BV(RBSM_EID_RC_LOST_SIGNAL);
            g_errors &= ~_BV(RBSM_EID_AUTON_LOST_SIGNAL);
        }

        // For the old buggy, the voltage divider is 10k ohm on the adc side and
        // 16k ohm on top.
        // Calculated map normally set to 13000, but the avcc is 4.86 volts
        // rather than 5.
        g_current_voltage = adc_read_blocking(BATTERY_ADC);
        g_current_voltage = map_signal(g_current_voltage, 0, 255, 0, 12636); // in millivolts

        // Read/convert steering pot
        g_steering_feedback = adc_read_blocking(STEERING_POT_ADC);
        g_steering_feedback = map_signal(g_steering_feedback,
                                         POT_OFFSET_STEERING_IN,
                                         POT_SCALE_STEERING_IN,
                                         PWM_OFFSET_STORED_ANGLE,
                                         PWM_SCALE_STORED_ANGLE);

    
        // Set outputs
        if(g_is_autonomous)
        {
            steering_set(auto_steering_angle);
            g_rbsm.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)(auto_steering_angle));
        }
        else
        {
            steering_set(steer_angle);
            g_rbsm.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)steer_angle);
        }
        
        if(brake_cmd_teleop_engaged == true ||
           (g_is_autonomous == true && brake_cmd_auton_engaged == true) ||
           brake_needs_reset == true)
        {
            brake_drop();
            g_rbsm.Send(RBSM_MID_MEGA_BRAKE_STATE,(long unsigned)true);
        } 
        else 
        {
            brake_raise();
            g_rbsm.Send(RBSM_MID_MEGA_BRAKE_STATE,(long unsigned)false);
        }

        // Send the rest of the telemetry messages
        g_rbsm.Send(RBSM_MID_DEVICE_ID, RBSM_DID_MEGA);
        g_rbsm.Send(RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND, (long unsigned)brake_cmd_teleop_engaged);
        g_rbsm.Send(RBSM_MID_MEGA_AUTON_BRAKE_COMMAND, (long unsigned)brake_cmd_auton_engaged);
        g_rbsm.Send(RBSM_MID_MEGA_AUTON_STATE, (long unsigned)g_is_autonomous);
        g_rbsm.Send(RBSM_MID_MEGA_BATTERY_LEVEL, g_current_voltage);
        g_rbsm.Send(RBSM_MID_MEGA_STEER_FEEDBACK, (long int)g_steering_feedback);
        g_rbsm.Send(RBSM_MID_ENC_TICKS_RESET, g_encoder_ticks_safe);
        g_rbsm.Send(RBSM_MID_ENC_TIMESTAMP, millis()); //TODO: Is this safe?
        g_rbsm.Send(RBSM_MID_COMP_HASH, (long unsigned)(FP_HEXCOMMITHASH));
        g_rbsm.Send(RBSM_MID_ERROR, g_errors);

        //Feed the watchdog to indicate things aren't timing out
        wdt_reset();
    
    } //End while(true)

    return 0;
}


ISR(WDT_INT)
{
    cli();
    brake_drop();
    while(1)
    {
    }
}

ISR(RX_STEERING_INT) 
{
    g_steering_rx.OnInterruptReceiver();
}


ISR(RX_BRAKE_INT) 
{
    g_brake_rx.OnInterruptReceiver();
}


ISR(RX_AUTON_INT) 
{
    g_auton_rx.OnInterruptReceiver();
}

ISR(ENCODER_INT) 
{
    unsigned long time_now = micros();
    g_encoder_ticks++;
}

