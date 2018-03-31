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

#include "../lib_avr/steering/Dynamixel_Serial.h"
#include "../lib_avr/rbserialmessages/rbserialmessages.h"
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

#define SERVO_ID 0x04
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
static int steer_angle;
static unsigned long g_errors;

RBSerialMessages g_rbsm;
ServoReceiver g_steering_rx;
RadioReceiver g_brake_rx;

UARTFILE g_uart_rbsm;
UARTFILE g_uart_debug;



// Functions declarations
// ADC
void adc_init(void);
int adc_read_blocking(uint8_t pin);

// Steering
void steering_set(long steering_angle_hundredths);

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

void steering_set(long steering_angle_hundredths)
{
    //Offset by 180 because going from positive to negative angles on servo will cause undesirable effects
    steering_angle_hundredths += 18000;
    //TODO: Magic number
    float ticks = steering_angle_hundredths/8.8;
    ticks = max(min(ticks, 0xFFF), 0);
    Dynamixel.servo(SERVO_ID, (unsigned int)ticks, 0x3FF); //0x3FF is max speed
}


int main(void) {
    // state variables
    bool brake_needs_reset = true; // 0 = nominal, !0 = needs reset
    bool brake_cmd_teleop_engaged = false;

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
    adc_init();
    brake_init();

    //TODO: Magic numbers
    Dynamixel.begin(57600);
    Dynamixel.setMode(4, 0x001, 0xFFF);

    // setup rbsm
    g_rbsm.Init(&g_uart_rbsm, &g_uart_rbsm);

    // set up rc receivers
    g_steering_rx.Init(&RX_STEERING_PIN, RX_STEERING_PINN, RX_STEERING_INTN);
    g_brake_rx.Init(&RX_BRAKE_PIN, RX_BRAKE_PINN, RX_BRAKE_INTN);

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
    while(1) 
    {
        // prepare to time the main loop
        unsigned long time_next_loop = micros() + STEERING_LOOP_TIME_US;

        // Check for RC commands
        steer_angle = g_steering_rx.GetAngleHundredths();
        brake_cmd_teleop_engaged = g_brake_rx.GetPulseWidth() > PWM_STATE_THRESHOLD;

        // Detect dropped radio conections

        unsigned long time1 = g_steering_rx.GetLastTimestamp();
        unsigned long time2 = g_brake_rx.GetLastTimestamp();
        //We need to call micros last because if we called it first, 
        //  time1 or time2could be greater than time_now if an interrupt
        //  fires and updates its timestamp and our delta values will underflow
        unsigned long time_now = micros();


        //RC time deltas
        unsigned long delta1 = time_now - time1;
        unsigned long delta2 = time_now - time2;

        bool rc_timeout = delta1 > CONNECTION_TIMEOUT_US ||
                          delta2 > CONNECTION_TIMEOUT_US;

        if(rc_timeout) 
        {
            // we haven't heard from the RC receiver in too long
            g_errors |= _BV(RBSM_EID_RC_LOST_SIGNAL);
            brake_needs_reset = true;
            dbg_printf("RC Timeout! %lu %lu %lu\n", delta1, delta2, delta3);
        }
        else 
        {
            if(brake_cmd_teleop_engaged == true) 
            {
                brake_needs_reset = false;
            }
            g_errors &= ~_BV(RBSM_EID_RC_LOST_SIGNAL);
        }     

        steering_set(steer_angle);
        g_rbsm.Send(RBSM_MID_MEGA_STEER_ANGLE, (long int)steer_angle);
        
        if (brake_cmd_teleop_engaged == true || brake_needs_reset == true) 
        {
            brake_drop();
            g_rbsm.Send(RBSM_MID_MEGA_BRAKE_STATE,(long unsigned)true);
        } 
        else 
        {
            brake_raise();
            g_rbsm.Send(RBSM_MID_MEGA_BRAKE_STATE,(long unsigned)false);
        }

        //TODO: Replace this block with the dynamixel command.
        // long steering_feedback_angle = map_signal(g_encoder_steering.GetTicks(),
        //                      0,
        //                      MOTOR_ENCODER_TICKS_PER_REV,
        //                      0,
        //                      DEGREE_HUNDREDTHS_PER_REV);

        // Send the rest of the telemetry messages
        g_rbsm.Send(DEVICE_ID, RBSM_DID_MEGA);
        g_rbsm.Send(RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND, (long unsigned)brake_cmd_teleop_engaged);
        //g_rbsm.Send(RBSM_MID_MEGA_STEER_FEEDBACK, steering_feedback_angle);
        //g_rbsm.Send(RBSM_MID_ENC_TICKS_RESET, g_encoder_distance.GetTicks());
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


