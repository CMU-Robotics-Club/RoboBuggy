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
#include <stdlib.h>
#include <avr/wdt.h>

#include "../lib_avr/encoder/encoder.h"
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
    #define STEERING_LIMIT_LEFT -1000   //steering_set assumes that left is less than right.
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

#define ENCODER_STEERING_DDR_A  DDRD
#define ENCODER_STEERING_PORT_A PORTD
#define ENCODER_STEERING_PIN_A  PIND
#define ENCODER_STEERING_PINN_A PD2
#define ENCODER_STEERING_INT_A  INT2_vect
#define ENCODER_STEERING_DDR_B  DDRD
#define ENCODER_STEERING_PORT_B PORTD
#define ENCODER_STEERING_PIN_B  PIND
#define ENCODER_STEERING_PINN_B PD3
#define ENCODER_STEERING_INT_B  INT3_vect

#define BATTERY_ADC 0
#define STEERING_POT_ADC 9

#define LED_DANGER_DDR  DDRB
#define LED_DANGER_PORT PORTB
#define LED_DANGER_PINN PB6 // arduino 12
#define BRAKE_OUT_DDR  DDRH
#define BRAKE_OUT_PORT PORTH
#define BRAKE_OUT_PINN PH5 // arduino 8
#define BRAKE_INDICATOR_DDR  DDRE
#define BRAKE_INDICATOR_PORT PORTE
#define BRAKE_INDICATOR_PINN PE3 // arduino 5

#define STEERING_LOOP_TIME_US 10000L
#define MICROSECONDS_PER_SECOND 1000000L
#define STEERING_KP_NUMERATOR -15L
#define STEERING_KP_DEMONENATOR 100L
#define STEERING_KD_NUMERATOR 0L
#define STEERING_KD_DENOMENATOR 100L
#define STEERING_MAX_SPEED 2000L //20 degress per second
#define STEERING_KV_NUMERATOR -15L
#define STEERING_KV_DENOMENATOR 100L
#define STEERING_PWM_CENTER 1500L //The PWM value that gives no movement.

#define MOTOR_ENCODER_TICKS_PER_REV 4096
#define DEGREE_HUNDREDTHS_PER_REV 36000

#define WDT_INT WDT_vect

//Uncomment the line below for testing. Otherwise leave commented out
#define DEBUG

#ifdef DEBUG
    #define dbg_printf(...) printf(__VA_ARGS__)
#else
    #define dbg_printf 
#endif

#define min(X,Y) ((X < Y) ? (X) : (Y))
#define max(X,Y) ((X > Y) ? (X) : (Y))


// Global state
static bool g_brake_state_engaged; // 0 = disengaged, !0 = engaged.
static bool g_brake_needs_reset; // 0 = nominal, !0 = needs reset
static bool g_is_autonomous;
static unsigned long g_current_voltage;
static unsigned long g_steering_feedback;
static int smoothed_thr;
static int smoothed_auton;
static int steer_angle;
static int auto_steering_angle;
static bool auto_brake_engaged;

RBSerialMessages g_rbsm;
rb_message_t g_new_rbsm;
ServoReceiver g_steering_rx;
ServoReceiver g_brake_rx;
ServoReceiver g_auton_rx;
Encoder g_encoder_distance;
Encoder g_encoder_steering;

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


long steer_set_prev_ticks = 0;
/*
 Sets the velocity of the steering motor to target_velocity.
 target_velocity is in hundredths of a degree per second.
 */
void steer_set_velocity(long target_velocity) {
	target_velocity = clamp(target_velocity, STEERING_MAX_SPEED, -(STEERING_MAX_SPEED));
	
	long current_ticks = g_encoder_steering.GetTicks();
	long change_in_ticks = current_ticks - steer_set_prev_ticks;
	dbg_printf("change in ticks: %ld\n", change_in_ticks);
	
	long change_in_angle = map_signal(change_in_ticks,
									  0,
									  MOTOR_ENCODER_TICKS_PER_REV,
									  0,
									  DEGREE_HUNDREDTHS_PER_REV);
	
	//angle * us/s * us = angle per second
	long actual_velocity = change_in_angle * (MICROSECONDS_PER_SECOND / STEERING_LOOP_TIME_US);
	long error = target_velocity - actual_velocity;
	
	long output_us = error * STEERING_KV_NUMERATOR / STEERING_KV_DENOMENATOR;
	
	//Send command to the motor
	servo_set_us(output_us + STEERING_PWM_CENTER);
	
	steer_set_prev_ticks = current_ticks;
}


long steer_set_error_prev = 0; //This is used to find the d term for position.

void steering_set(int angle) //TODO: should angle be an int or a long?
{
	//TODO: delete short circuiting code for testing velocity measure
	steer_set_velocity(output_vel);
	return;
	
	angle = clamp(angle, STEERING_LIMIT_RIGHT, STEERING_LIMIT_LEFT);
	
    //For the DC motor
    long feed_forward = -10;
    long actual = map_signal(g_encoder_steering.GetTicks(),
                             0,
                             MOTOR_ENCODER_TICKS_PER_REV,
                             0,
                             DEGREE_HUNDREDTHS_PER_REV);
	
    long error = angle - actual;
    long output_vel = 0;
    if(labs(error) > 10) { //0.2 degree deadband
        long output_p = STEERING_KP_NUMERATOR * error / STEERING_KP_DEMONENATOR;
        
        long output_ff = (error > 0) ? feed_forward : -feed_forward;
        
        long d =  (error - steer_set_error_prev); //not dividing by time
        long output_d = STEERING_KD_NUMERATOR * d / STEERING_KD_DENOMENATOR;
        output_vel = output_p + output_ff + output_d;
    }

    steer_set_error_prev = error;

    dbg_printf("actual: %ld, ticks: %ld\r\n", actual, g_encoder_steering.GetTicks());
    dbg_printf("output_vel: %ld\r\n", output_vel);
    steer_set_velocity(output_vel);

    //Meant for a servo
    // angle = clamp(angle, 
    //               STEERING_LIMIT_RIGHT,
    //               STEERING_LIMIT_LEFT);

    // int servo_value_us = map_signal(angle,
    //                                 PWM_OFFSET_STORED_ANGLE,
    //                                 PWM_SCALE_STORED_ANGLE,
    //                                 PWM_OFFSET_STEERING_OUT,
    //                                 PWM_SCALE_STEERING_OUT);
    // servo_set_us(servo_value_us);
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
    // turn the ledPin on
    // DEBUG_PORT |= _BV(DEBUG_PINN);
    DEBUG_PORT &= ~_BV(DEBUG_PINN);
    DEBUG_DDR |= _BV(DEBUG_PINN);

    // setup encoder pin and interrupt
    // ENCODER_PORT |= _BV(ENCODER_PINN);
    // ENCODER_DDR &= ~_BV(ENCODER_PINN); 
    EIMSK |= _BV(INT2);
    EICRA |= _BV(ISC20);
    EICRA &= ~_BV(ISC21);
    EIMSK |= _BV(INT3);
    EICRA |= _BV(ISC30);
    EICRA &= ~_BV(ISC31);
    // g_encoder_distance.Init(&ENCODER_PIN, ENCODER_PINN);
    g_encoder_steering.InitQuad(&ENCODER_STEERING_PIN_A,
                                ENCODER_STEERING_PINN_A,
                                &ENCODER_STEERING_PIN_B,
                                ENCODER_STEERING_PINN_B);

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
    LED_DANGER_DDR |= _BV(LED_DANGER_PINN);

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
    dbg_printf("Hello world! This is debug information\r\n");
    dbg_printf("Compilation date: %s\r\n", FP_COMPDATE);
    dbg_printf("Compilation time: %s\r\n", FP_COMPTIME);
    dbg_printf("Branch name: %s\r\n", FP_BRANCHNAME);
    dbg_printf("Most recent commit: %s\r\n", FP_STRCOMMITHASH);
    dbg_printf("Branch clean? %d\r\n", FP_CLEANSTATUS);
    dbg_printf("\nEnd of compilation information\r\n");

    // prepare to for timeouts
    unsigned long time_start = micros();
    unsigned long auton_brake_last = time_start;
    unsigned long auton_steer_last = time_start;

    //false = brake dropped, true = brake engaged
    bool teleop_brake_command = true;
    bool auton_brake_command = true;

    watchdog_init();

    // loop forever
    while(1) 
    {
        // prepare to time the main loop
        unsigned long time_next_loop = micros() + STEERING_LOOP_TIME_US;
        DEBUG_PORT &= ~_BV(DEBUG_PINN);

        // enable servo power after timeout
        if(millis() > 200) 
        {
            PORTL |= _BV(0);
        }

        rb_message_t new_command;
        int read_status;

        while((read_status = g_rbsm.Read(&new_command))
             != RBSM_ERROR_INSUFFICIENT_DATA) 
        {
            if(read_status == 0) 
            {
                // dipatch complete message
                switch(new_command.message_id)
                {
                    case RBSM_MID_ENC_RESET_REQUEST:
                        g_encoder_distance.Reset();
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
                        auto_brake_engaged = (bool)(long)new_command.data;
                        auton_brake_last = micros();
                        dbg_printf("Got brake message for %d.\n", auto_brake_engaged);
                        break;
                    default:
                        // report unknown message
                        g_rbsm.Send(RBSM_MID_ERROR, RBSM_EID_RBSM_INVALID_MID);
                        dbg_printf("Got message with invalid mid %d and data %lu\n",
                             new_command.message_id,
                             new_command.data);
                        break;
                } //End switch(new_command.message_id)
            } 
            else if(read_status == RBSM_ERROR_INVALID_MESSAGE) 
            {
                // report stream losses for tracking
                g_rbsm.Send(RBSM_MID_ERROR, RBSM_EID_RBSM_LOST_STREAM);
            }
            // drop responses with other faults
        }

        // find the new steering angle, if available
        steer_angle = g_steering_rx.GetAngleHundredths();

        // find the new brake state, if available
        smoothed_thr = g_brake_rx.GetAngle();

        teleop_brake_command = smoothed_thr > PWM_STATE_THRESHOLD;

        // TODO make this code...less...something
        if(teleop_brake_command) 
        {
            // read as engaged
            g_brake_state_engaged = true;
            // brake has been reset
            g_brake_needs_reset = false;
        } 
        else 
        {
            // read as disengaged
            g_brake_state_engaged = false;
        }

        // find the new autonomous state, if available
        smoothed_auton = g_auton_rx.GetAngle();

        // TODO make this code...less...something
        if(smoothed_auton > PWM_STATE_THRESHOLD) 
        { 
            // read as engaged
            g_is_autonomous = true;
        } 
        else 
        {
            // read as disengaged
            g_is_autonomous = false;
        }

        // detect dropped radio conections
        // note: interrupts must be disabled while checking system clock so that
        //       timestamps are not updated under our feet
        cli(); //disable interrupts
        unsigned long time_now = micros();
        unsigned long time1 = g_steering_rx.GetLastTimestamp();
        unsigned long time2 = g_brake_rx.GetLastTimestamp();
        unsigned long time3 = g_auton_rx.GetLastTimestamp();
        sei(); //enable interrupts
        //RC time deltas
        unsigned long delta1 = time_now - time1;
        unsigned long delta2 = time_now - time2;
        unsigned long delta3 = time_now - time3;
        //Auton time deltas
        unsigned long delta5 = time_now - auton_steer_last;
        unsigned long delta4 = time_now - auton_brake_last;

        if(delta1 > CONNECTION_TIMEOUT_US ||
           delta2 > CONNECTION_TIMEOUT_US ||
           delta3 > CONNECTION_TIMEOUT_US)
        {
            // we haven't heard from the RC receiver in too long
            if(g_brake_needs_reset == false) 
            {
                g_rbsm.Send(RBSM_MID_ERROR, RBSM_EID_RC_LOST_SIGNAL);
            }
            g_brake_needs_reset = true;
        }

        if((g_is_autonomous) && 
           (delta4 > CONNECTION_TIMEOUT_US ||
           delta5 > CONNECTION_TIMEOUT_US))
        {
            if(g_brake_needs_reset == false) 
            {
                g_rbsm.Send(RBSM_MID_ERROR, RBSM_EID_AUTON_LOST_SIGNAL);
            }
            g_brake_needs_reset = true;
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

        g_is_autonomous = true;
        if(g_is_autonomous)
        {
            if(auto_brake_engaged)
            {
                g_brake_state_engaged = true;                
            }

            steering_set(auto_steering_angle);
            g_rbsm.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)(auto_steering_angle));
        }
        else
        {
            steering_set(steer_angle);
            g_rbsm.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)steer_angle);
        }
        
        // Set outputs
        if(g_brake_state_engaged == false && g_brake_needs_reset == false) 
        {
            brake_raise();
        } 
        else 
        {
            brake_drop();
        }

        if(g_brake_needs_reset == true) 
        {
            LED_DANGER_PORT |= _BV(LED_DANGER_PINN);
        } 
        else 
        {
            LED_DANGER_PORT &= ~_BV(LED_DANGER_PINN);
        }

        // Send the rest of the telemetry messages
        g_rbsm.Send(RBSM_MID_DEVICE_ID, RBSM_DID_MEGA);
        g_rbsm.Send(RBSM_MID_MEGA_BRAKE_STATE,(long unsigned)g_brake_state_engaged);
        g_rbsm.Send(RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND, (long unsigned)teleop_brake_command);
        g_rbsm.Send(RBSM_MID_MEGA_AUTON_BRAKE_COMMAND, (long unsigned)auton_brake_command);
        g_rbsm.Send(RBSM_MID_MEGA_AUTON_STATE, (long unsigned)g_is_autonomous);
        g_rbsm.Send(RBSM_MID_MEGA_BATTERY_LEVEL, g_current_voltage);
        g_rbsm.Send(RBSM_MID_MEGA_STEER_FEEDBACK, (long int)g_steering_feedback);
        g_rbsm.Send(RBSM_MID_ENC_TICKS_RESET, g_encoder_steering.GetTicks());
        g_rbsm.Send(RBSM_MID_ENC_TIMESTAMP, millis());
        g_rbsm.Send(RBSM_MID_COMP_HASH, (long unsigned)(FP_HEXCOMMITHASH));

        DEBUG_PORT |= _BV(DEBUG_PINN);
        // wait for the next regular loop time in a tight loop
        // note: this is before the watchdog reset so it can catch us if
        // something goes wrong
        if(time_next_loop < micros()) {
            // TODO: throw loop timing error
        }
        while(time_next_loop > micros()) {}

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
    g_encoder_steering.OnInterruptQuad();
    // g_encoder_distance.OnInterrupt();
}

ISR(INT3_vect) {
    g_encoder_steering.OnInterruptQuad();
}

