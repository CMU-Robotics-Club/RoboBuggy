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

#include "../lib_avr/encoder/Rotary.h"
#include "../lib_avr/encoder/Quadrature.h"
#include "../lib_avr/rbserialmessages/rbserialmessages.h"
#include "../lib_avr/statuslights/StatusLights.h"
#include "../lib_avr/radioreceiver/RadioReceiver.h"
#include "../lib_avr/radioreceiver/ServoReceiver.h"
#include "../lib_avr/uart/uart_extra.h"
#include "servo.h"
#include "system_clock.h"
#include "fingerprint.h"

// Use 57600 so that the jetson and linux can recognize it because they need standard baudrate
// and since 115200 is too high for the 16 Mhz clock
// ATMega 2560 datasheet Table 22-12.
#define BAUD 57600

#define CONNECTION_TIMEOUT_US 1000000L // 1000ms
#define SYSTEM_VOLTAGE_THRESHOLD 12000 //For Buggy battery voltage 

/*
 * These values map the physical input/output (voltage/ms of pwm pulse) to a
 * fixed physical angle. When software commands a steering angle of 10 deg
 * (1000 hundredths), the PWM_SCALE_STEERING_OUT
 * should be adjusted until a 10 deg input/output is seen/observed.
 */
#define PWM_OFFSET_STORED_ANGLE 0
#define PWM_SCALE_STORED_ANGLE 1000 // in hundredths of a degree for precision

#define STEERING_LIMIT_LEFT -1500   //steering_set assumes that left is less than right.
#define STEERING_LIMIT_RIGHT 1500

//Off/down state is 1500 +- 20 microseconds, on/up state is 2000 +- 20, 
//  so 1750 serves as a fair threshold value to deliminate the two
#define PWM_STATE_THRESHOLD 1750

#define RX_STEERING_PIN  PINE
#define RX_STEERING_PINN PE4 // arduino 2
#define RX_STEERING_INT  INT4_vect
#define RX_STEERING_INTN 4

#define RX_BRAKE_PIN  PIND
#define RX_BRAKE_PINN PD0 // arduino 21
#define RX_BRAKE_INT  INT0_vect
#define RX_BRAKE_INTN 0

#define RX_AUTON_PIN  PIND
#define RX_AUTON_PINN PD1 // arduino 20
#define RX_AUTON_INT  INT1_vect
#define RX_AUTON_INTN 1

#define STATUS_LIGHT_PORT_GREEN PORTE
#define STATUS_LIGHT_PINN_GREEN PE3
#define STATUS_LIGHT_DDR_GREEN DDRE
#define STATUS_LIGHT_PORT_BLUE PORTE
#define STATUS_LIGHT_PINN_BLUE PE5
#define STATUS_LIGHT_DDR_BLUE DDRE
#define STATUS_LIGHT_PORT_RED PORTH
#define STATUS_LIGHT_PINN_RED PH6
#define STATUS_LIGHT_DDR_RED DDRH

#define ENCODER_INT  INT2_vect

#define ENCODER_STEERING_A_DDR  DDRB
#define ENCODER_STEERING_A_PORT PORTB
#define ENCODER_STEERING_A_PIN  PINB
#define ENCODER_STEERING_A_PINN PB4
#define ENCODER_STEERING_B_DDR  DDRB
#define ENCODER_STEERING_B_PORT PORTB
#define ENCODER_STEERING_B_PIN  PINB
#define ENCODER_STEERING_B_PINN PB6
#define ENCODER_STEERING_PCINT  PCINT0_vect

// Constants obtained by measuring the sensor ADC value as function of
// applied voltage, scanning the full range and then doing a linear regression
#define BATTERY_ADC 0 // pin to read battery level from (A0)
#define BATTERY_ADC_SLOPE  14.683 // in mV, obtained from calibration
#define BATTERY_ADC_OFFSET 44.359

#define BRAKE_OUT_DDR  DDRH
#define BRAKE_OUT_PORT PORTH
#define BRAKE_OUT_PINN PH5 // arduino 8

#define STEERING_LOOP_TIME_US 10000L
#define MICROSECONDS_PER_SECOND 1000000L
#define STEERING_KP_NUMERATOR 13L
#define STEERING_KP_DEMONENATOR 1L
#define STEERING_KD_NUMERATOR 0L
#define STEERING_KD_DENOMENATOR 100L
//limited by 16-bit signed? 400 degress per second - 50% cpu usage from encoder interrupts
#define STEERING_MAX_SPEED 32000L 
#define STEERING_KV_NUMERATOR 3L
#define STEERING_KV_DENOMENATOR 1000L
#define STEERING_PWM_CENTER_US 1500L //The PWM value that gives no movement.
#define STEERING_MAX_PWM 150 // max PWM value to add on top of the center_us
#define MOTOR_ENCODER_TICKS_PER_REV 36864 // 4096 * 9 = 12-bit * 1:9 gearbox
#define MOTOR_FEED_FORWARD 200
#define STEERING_ERROR_THRESHOLD 5
#define DEGREE_HUNDREDTHS_PER_REV 36000
#define STEERING_CENTER_DDR  DDRH
#define STEERING_CENTER_PORT PORTH
#define STEERING_CENTER_PIN  PINH
#define STEERING_CENTER_PINN PH3
#define STEERING_CENTER_SEARCH_V 800 // hundredths of deg/s

#define WDT_INT WDT_vect

//Uncomment the line below for testing. Otherwise leave commented out
// #define DEBUG

#ifdef DEBUG
    #define dbg_printf(...) printf(__VA_ARGS__)
#else
    #define dbg_printf(...)
#endif

#define min(X,Y) ((X < Y) ? (X) : (Y))
#define max(X,Y) ((X > Y) ? (X) : (Y))


// Global state
static bool g_is_autonomous;
static unsigned long g_current_voltage; // in mV
static int steer_angle;
static int auto_steering_angle;
static unsigned long g_errors;

RBSerialMessages g_rbsm;
ServoReceiver g_steering_rx;
RadioReceiver g_brake_rx;
RadioReceiver g_auton_rx;
StatusLights lights_rc(STATUS_LIGHT_PINN_GREEN, 
                       &STATUS_LIGHT_PORT_GREEN, 
                       &STATUS_LIGHT_DDR_GREEN);
StatusLights lights_auton(STATUS_LIGHT_PINN_RED, 
                          &STATUS_LIGHT_PORT_RED, 
                          &STATUS_LIGHT_DDR_RED);
StatusLights lights_battery(STATUS_LIGHT_PINN_BLUE, 
                            &STATUS_LIGHT_PORT_BLUE, 
                            &STATUS_LIGHT_DDR_BLUE);
Rotary g_encoder_distance;
Quadrature g_encoder_steering;

UARTFILE g_uart_rbsm;
UARTFILE g_uart_debug;



// Functions declarations
// ADC
void adc_init(void);
int adc_read_blocking(uint8_t pin);

// Steering
void steer_set_velocity(long target_velocity);
void steering_set(int angle);
int8_t steering_center();

// Brakes
void brake_init();
void brake_raise();
void brake_drop();

// Watchdog
void watchdog_init();



inline long map_signal(long x,
                       long in_offset,
                       long in_scale,
                       long out_offset,
                       long out_scale) {
    return ((x - in_offset) * out_scale / in_scale) + out_offset;
}

inline long clamp(long input,
                  long upper,
                  long lower) {
    return (max(min(input, upper), lower));
}


void adc_init(void) { // copy-pasted from wiring.c l.353 (Arduino library)
    // set a2d prescaler so we are inside the desired 50-200 KHz range.
    sbi(ADCSRA, ADPS2);
    sbi(ADCSRA, ADPS1);
    sbi(ADCSRA, ADPS0);
    // enable a2d conversions
    sbi(ADCSRA, ADEN);
}

int adc_read_blocking(uint8_t pin) { // takes less than 160us, return 0 to 1023
    uint8_t low, high;

    // the MUX5 bit of ADCSRB selects whether we're reading from channels
    // 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);

    // set the analog reference (high two bits of ADMUX) and select the
    // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
    // to 0 (the default).
    ADMUX = (1 << 6) | (pin & 0x07);

    // start the conversion
    sbi(ADCSRA, ADSC);

    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC));

    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    low  = ADCL;
    high = ADCH;

    // combine the two bytes
    return (high << 8) | low;
}


/** @brief sets the velocity of the steering motor to target_velocity
 *
 *  @param target_velocity is in hundredths of a degree per second. values > 0
 *      drive to the right
 */
void steer_set_velocity(long target_velocity) {
    static long steer_set_prev_ticks = 0;
    static long steer_set_prev_velocity = 0;

    target_velocity = clamp(target_velocity, STEERING_MAX_SPEED, -(STEERING_MAX_SPEED));
    
    long current_ticks = g_encoder_steering.GetTicks();
    long change_in_ticks = current_ticks - steer_set_prev_ticks;
    long change_in_angle = (change_in_ticks * DEGREE_HUNDREDTHS_PER_REV) / DEGREE_HUNDREDTHS_PER_REV;
    //angle * us/s * us = angle per second
    long actual_velocity = (change_in_angle * MICROSECONDS_PER_SECOND) / STEERING_LOOP_TIME_US;
    
    long error = target_velocity - actual_velocity;
    
    long output_us = 0;
    long output_p = error * STEERING_KV_NUMERATOR / STEERING_KV_DENOMENATOR;
    output_us = output_p;
    long output_int_us = steer_set_prev_velocity + output_us;
    output_int_us = clamp(output_int_us, STEERING_MAX_PWM, -STEERING_MAX_PWM);
        
    // dbg_printf("target: %ld, current: %ld, error: %ld, correction: %ld, output: %ld\n",
    // target_velocity, actual_velocity, error, output_us, output_int_us);
    
    //Send command to the motor
    servo_set_us(output_int_us + STEERING_PWM_CENTER_US);
    
    steer_set_prev_ticks = current_ticks;
    steer_set_prev_velocity = output_int_us;
}


/** @brief sets the steering angle using PID feedback
 *
 *  @param angle set point in hundredths of a degree
 */
void steering_set(int angle)
{
    static long steer_set_error_prev = 0; //This is used to find the d term for position.

    angle = clamp(angle, STEERING_LIMIT_RIGHT, STEERING_LIMIT_LEFT);
    
    //For the DC motor
    long actual = map_signal(g_encoder_steering.GetTicks(),
                             0,
                             MOTOR_ENCODER_TICKS_PER_REV,
                             0,
                             DEGREE_HUNDREDTHS_PER_REV);
    
    long error = angle - actual;
    long output_vel = 0;
    if(labs(error) > STEERING_ERROR_THRESHOLD) { //0.1 degree deadband
        long output_p = (STEERING_KP_NUMERATOR * error) / STEERING_KP_DEMONENATOR;
        long output_ff = (error > 0) ? MOTOR_FEED_FORWARD : -MOTOR_FEED_FORWARD;
        long d =  (error - steer_set_error_prev); //not dividing by time
        long output_d = (STEERING_KD_NUMERATOR * d) / STEERING_KD_DENOMENATOR;
        output_vel = output_p + output_ff + output_d;
    }

    steer_set_error_prev = error;

    dbg_printf("actual: %ld, error: %ld, output_vel: %ld\n", actual, error, output_vel);
    steer_set_velocity(output_vel);
}


/** @brief Uses the steering center edge finder to detect the drive center.
 *
 *  This call may block for a while, but not more than the timeout based on
 *  the search velocity and steering limits. We always search for the edge where
 *  the sensor goes from no metal to metal detected so we aren't effected by
 *  hysteresis of the system.
 */
int8_t steering_center() {
    // setup center finder pin for pull up input
    STEERING_CENTER_DDR &= ~_BV(STEERING_CENTER_PINN);
    STEERING_CENTER_PORT |= _BV(STEERING_CENTER_PINN);
    // calculate max search time
    int16_t search_loop_count = (STEERING_LIMIT_RIGHT * MICROSECONDS_PER_SECOND)
                                 / (STEERING_CENTER_SEARCH_V * STEERING_LOOP_TIME_US);
    // sensor is active low
    bool steering_center_tripped;
    uint32_t time_next_loop;

    // if sensor is already tripped, move left first to clear
    for(int16_t i = search_loop_count; i > 0; i--) {
        time_next_loop = micros() + STEERING_LOOP_TIME_US;
        steering_center_tripped = !(STEERING_CENTER_PIN & _BV(STEERING_CENTER_PINN));
        if(!steering_center_tripped) {
            break;
        }
        steer_set_velocity(-STEERING_CENTER_SEARCH_V);
        // wait for next loop
        while(time_next_loop > micros()) {}
    }
    steer_set_velocity(0);

    // now find the edge moving to the right
    for(int16_t i = search_loop_count; i > 0; i--) {
        time_next_loop = micros() + STEERING_LOOP_TIME_US;
        steering_center_tripped = !(STEERING_CENTER_PIN & _BV(STEERING_CENTER_PINN));
        if(steering_center_tripped) {
            break;
        }
        steer_set_velocity(STEERING_CENTER_SEARCH_V);
        // wait for next loop
        while(time_next_loop > micros()) {}
    }
    steer_set_velocity(0);

    // reset encoder with this positon
    g_encoder_steering.Reset();
    return 0;
}


void brake_init() {
    BRAKE_OUT_DDR |= _BV(BRAKE_OUT_PINN);
}


// Note: High-voltage raises the brake.
// Do not call before brake_init
void brake_raise() 
{
    BRAKE_OUT_PORT |= _BV(BRAKE_OUT_PINN);
}


// Drops the brake
// Do not call before brake_init
void brake_drop() {
    BRAKE_OUT_PORT &= ~_BV(BRAKE_OUT_PINN);
}


/*
* Function: watchdog_init
*
* Description: Configures the hardware watchdog to monitor for timeouts.
*   Sets the system to "bark" after 1 second without resets.  Has an 
*   independent clock and will check for timeout regardless of main code.
*/
void watchdog_init() {
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


int main(void) {
    // state variables
    bool brake_needs_reset = true; // 0 = nominal, !0 = needs reset
    bool brake_cmd_teleop_engaged = false;
    bool brake_cmd_auton_engaged = false;
    unsigned long time_start = micros();
    unsigned long auton_brake_last = time_start;
    unsigned long auton_steer_last = time_start;

    g_encoder_distance.Init();

    g_encoder_steering.InitQuad(&ENCODER_STEERING_A_PIN,
                                ENCODER_STEERING_A_PINN,
                                &ENCODER_STEERING_B_PIN,
                                ENCODER_STEERING_B_PINN,
                                &ENCODER_STEERING_A_PORT,
                                &ENCODER_STEERING_A_DDR,
                                &ENCODER_STEERING_B_PORT,
                                &ENCODER_STEERING_B_DDR);

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
    steering_center(); // this call takes time

    // setup rbsm
    g_rbsm.Init(&g_uart_rbsm, &g_uart_rbsm);

    // set up rc receivers
    g_steering_rx.Init(&RX_STEERING_PIN, RX_STEERING_PINN, RX_STEERING_INTN);
    g_brake_rx.Init(&RX_BRAKE_PIN, RX_BRAKE_PINN, RX_BRAKE_INTN);
    g_auton_rx.Init(&RX_AUTON_PIN, RX_AUTON_PINN, RX_AUTON_INTN);

    //Output information about code on arduino once to debug uart on startup.
    printf("Hello world! This is debug information\r\n");
    printf("Compilation date: %s\r\n", FP_COMPDATE);
    printf("Compilation time: %s\r\n", FP_COMPTIME);
    printf("Branch name: %s\r\n", FP_BRANCHNAME);
    printf("Most recent commit: %s\r\n", FP_STRCOMMITHASH);
    printf("Branch clean? %d\r\n", FP_CLEANSTATUS);
    printf("\nEnd of compilation information\r\n");

    watchdog_init();

    // loop forever
    while(1) {
        // prepare to time the main loop
        unsigned long time_next_loop = micros() + STEERING_LOOP_TIME_US;

        // Check for incomming serial messages
        rb_message_t new_command;
        int read_status;

        while((read_status = g_rbsm.Read(&new_command))
              != RBSM_ERROR_INSUFFICIENT_DATA) {
            if(read_status == 0) {
                // clear RBSM errors
                g_errors &= ~_BV(RBSM_EID_RBSM_LOST_STREAM);
                g_errors &= ~_BV(RBSM_EID_RBSM_INVALID_MID);
                // dipatch complete message
                switch(new_command.message_id) {
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
                        brake_cmd_auton_engaged = (bool)(long)new_command.data;
                        auton_brake_last = micros();
                        dbg_printf("Got brake message for %d.\n", brake_cmd_auton_engaged);
                        break;
                    default:
                        // report unknown message
                        dbg_printf("Got message with invalid mid %d and data %lu\n",
                                   new_command.message_id,
                                   new_command.data);
                        g_errors |= _BV(RBSM_EID_RBSM_INVALID_MID);
                        break;
                } //End switch(new_command.message_id)
            } else if(read_status == RBSM_ERROR_INVALID_MESSAGE) {
                // report stream losses for tracking
                dbg_printf("RBSM could not parse message.\n");
                g_errors |= _BV(RBSM_EID_RBSM_LOST_STREAM);
            } else {
                // should not be other faults
                dbg_printf("Unknown RBSM parse error.\n");
                g_errors |= _BV(RBSM_EID_RBSM_LOST_STREAM);
            }
        }

        // Check for RC commands
        steer_angle = g_steering_rx.GetAngleHundredths();
        brake_cmd_teleop_engaged = g_brake_rx.GetPulseWidth() > PWM_STATE_THRESHOLD;
        g_is_autonomous = g_auton_rx.GetPulseWidth() > PWM_STATE_THRESHOLD;

        // Detect dropped radio conections

        unsigned long time1 = g_steering_rx.GetLastTimestamp();
        unsigned long time2 = g_brake_rx.GetLastTimestamp();
        unsigned long time3 = g_auton_rx.GetLastTimestamp();
        //We need to call micros last because if we called it first, 
        //  time1, time2, or time3 could be greater than time_now if an interrupt
        //  fires and updates its timestamp and our delta values will underflow
        unsigned long time_now = micros();


        //RC time deltas
        unsigned long delta1 = time_now - time1;
        unsigned long delta2 = time_now - time2;
        unsigned long delta3 = time_now - time3;
        //Auton time deltas
        unsigned long delta5 = time_now - auton_steer_last;
        unsigned long delta4 = time_now - auton_brake_last;

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
                g_errors |= _BV(RBSM_EID_RC_LOST_SIGNAL);
                brake_needs_reset = true;
                lights_rc.Enable();
                dbg_printf("RC Timeout! %lu %lu %lu\n", delta1, delta2, delta3);
            } else {
                g_errors &= ~_BV(RBSM_EID_RC_LOST_SIGNAL);
            }

            // then check for timeout under autonomous
            if(auton_timeout) {
                g_errors |= _BV(RBSM_EID_AUTON_LOST_SIGNAL);
                brake_needs_reset = true;
                lights_auton.Enable();
                dbg_printf("Auton Timeout! %lu %lu\n", delta4, delta5);
            } else {
                g_errors &= ~_BV(RBSM_EID_AUTON_LOST_SIGNAL);
            }
        // or reset the system if connection is back and driver engages brakes
        } else { 
            if(brake_cmd_teleop_engaged == true) {
                brake_needs_reset = false;
            }

            g_errors &= ~_BV(RBSM_EID_RC_LOST_SIGNAL);
            g_errors &= ~_BV(RBSM_EID_AUTON_LOST_SIGNAL);
            // once the connection is back can turn off lights
            lights_auton.Disable();
            lights_rc.Disable();
        }

        // Reading battery voltage from the voltage 24/12 kOhm divider
        g_current_voltage  = adc_read_blocking(BATTERY_ADC);
        g_current_voltage *= BATTERY_ADC_SLOPE;
        g_current_voltage += BATTERY_ADC_OFFSET;
        
        if (g_current_voltage < SYSTEM_VOLTAGE_THRESHOLD) {
            lights_battery.Enable();
        } else {
            lights_battery.Disable();
        }
        
        // Set outputs
        if(g_is_autonomous) {
            steering_set(auto_steering_angle);
            g_rbsm.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)(auto_steering_angle));
        } else {
            steering_set(steer_angle);
            g_rbsm.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)steer_angle);
        }
        
        if(brake_cmd_teleop_engaged == true ||
           (g_is_autonomous == true && brake_cmd_auton_engaged == true) ||
           brake_needs_reset == true) {
            brake_drop();
            g_rbsm.Send(RBSM_MID_MEGA_BRAKE_STATE,(long unsigned)true);
        } else {
            brake_raise();
            g_rbsm.Send(RBSM_MID_MEGA_BRAKE_STATE,(long unsigned)false);
        }

        long steering_feedback_angle = map_signal(g_encoder_steering.GetTicks(),
                             0,
                             MOTOR_ENCODER_TICKS_PER_REV,
                             0,
                             DEGREE_HUNDREDTHS_PER_REV);

        // Send the rest of the telemetry messages
        g_rbsm.Send(DEVICE_ID, RBSM_DID_MEGA);
        g_rbsm.Send(RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND, (long unsigned)brake_cmd_teleop_engaged);
        g_rbsm.Send(RBSM_MID_MEGA_AUTON_BRAKE_COMMAND, (long unsigned)brake_cmd_auton_engaged);
        g_rbsm.Send(RBSM_MID_MEGA_AUTON_STATE, (long unsigned)g_is_autonomous);
        g_rbsm.Send(RBSM_MID_MEGA_BATTERY_LEVEL, g_current_voltage);
        g_rbsm.Send(RBSM_MID_MEGA_STEER_FEEDBACK, steering_feedback_angle);
        g_rbsm.Send(RBSM_MID_ENC_TICKS_RESET, g_encoder_distance.GetTicks());
        g_rbsm.Send(RBSM_MID_MEGA_TIMESTAMP, millis());
        g_rbsm.Send(RBSM_MID_COMP_HASH, (long unsigned)(FP_HEXCOMMITHASH));
        g_rbsm.Send(RBSM_MID_ERROR, g_errors);

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


ISR(WDT_INT) {
    cli();
    brake_drop();
    while(1)
    {
    }
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

ISR(ENCODER_INT) {
    g_encoder_distance.OnInterrupt();
}

ISR(ENCODER_STEERING_PCINT) {
    g_encoder_steering.OnInterrupt();
}

