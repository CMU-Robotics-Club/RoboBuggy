#include "encoder.h"


Encoder::Encoder() {
}


uint8_t Encoder::Init(volatile uint8_t *pin_a_reg,
                      uint8_t pin_a_num,
                      volatile uint8_t *port_reg,
                      volatile uint8_t *ddr_reg) {
    // save configuration info
    pin_a_reg_ = pin_a_reg;
    pin_a_num_ = pin_a_num;

    // reset state
    ticks_ = 0;
    errors_ = 0;

    // setup encoder pin with pullups and interrupt
    *port_reg |= _BV(pin_a_num);
    *ddr_reg &= ~_BV(pin_a_num);

    //TODO: Too hard coded?  Maybe turn into input arguments?
    //External interrupt mask register. Enables external interrupt on pin PD2
    EIMSK |= _BV(INT2);
    //External interrupt control register A.  Configures interrupt to be triggered on any edge
    EICRA |= _BV(ISC20);
    EICRA &= ~_BV(ISC21);

    return errors_;
}


uint8_t Encoder::InitQuad(volatile uint8_t *pin_a_reg,
                          uint8_t pin_a_num,
                          volatile uint8_t *pin_b_reg,
                          uint8_t pin_b_num,
                          volatile uint8_t *port_a_reg,
                          volatile uint8_t *ddr_a_reg,
                          volatile uint8_t *port_b_reg,
                          volatile uint8_t *ddr_b_reg) {
    // save configuration info
    pin_a_reg_ = pin_a_reg;
    pin_a_num_ = pin_a_num;
    pin_b_reg_ = pin_b_reg;
    pin_b_num_ = pin_b_num;
    // reset state
    ticks_ = 0;
    bool pin_a = (*pin_a_reg_) & _BV(pin_a_num_);
    bool pin_b = (*pin_b_reg_) & _BV(pin_b_num_);
    pin_state_last_ = ((uint8_t)pin_a << 1) | ((uint8_t)pin_b);
    errors_ = 0;

    // setup steering encoder on pcint 0 with pullups off
    *port_a_reg &= ~_BV(pin_a_num);
    *ddr_a_reg &= ~_BV(pin_a_num);
    *port_b_reg &= ~_BV(pin_b_num);
    *ddr_b_reg &= ~_BV(pin_b_num);

    //Pin change mask register
    PCMSK0 |= _BV(PCINT4) | _BV(PCINT6);
    //Pin change interrupt control register
    PCICR |= _BV(PCIE0);

    return errors_;
}


void Encoder::OnInterrupt() {
    ticks_++;
    return;
}


void Encoder::OnInterruptQuad() {
    // read in new pin values
    bool pin_a_new = (*pin_a_reg_) & _BV(pin_a_num_);
    bool pin_b_new = (*pin_b_reg_) & _BV(pin_b_num_);

    // match last cycle + this cycle values to lookup table
    uint8_t pin_state = ((pin_state_last_ << 2)
                         | ((uint8_t)pin_a_new << 1) | ((uint8_t)pin_b_new))
                        & 0x0F;
    switch(pin_state) {
        /*
                                  _______         _______
                      PinA ______|       |_______|       |______ PinA
        forward  <---         _______         _______         __      --> reverse
                      PinB __|       |_______|       |_______|   PinB

        old A B new A B
        0000 (0): no change
        0001 (1): reverse
        0010 (2): forward
        0011 (3): reverse x2 (assume A interrupt only)
        0100 (4): forward
        0101 (5): no change
        0110 (6): forward x2 (assume A interrupt only)
        0111 (7): reverse
        1000 (8): reverse
        1001 (9): forward x2 (assume A interrupt only)
        1010 (10): no change
        1011 (11): forward
        1100 (12): reverse x2 (assume A interrupt only)
        1101 (13): forward
        1110 (14): reverse
        1111 (15): no change

        Similar algorithm to PJRC and circuitsathome.com.
        */

        // forward
        case 2:
        case 4:
        case 11:
        case 13:
            ticks_++;
            break;

        // reverse
        case 1:
        case 7:
        case 8:
        case 14:
            ticks_--;
            break;

        #ifdef ENCODER_ALLOW_SINGLE_INT
        // missed tick
        case 6:
        case 9:
            errors_ |= _BV(ENCODER_ERROR_MISSED_TICK);
            return;
            break;
        case 3:
        case 12:
            errors_ |= _BV(ENCODER_ERROR_MISSED_TICK);
            return;
            break;
        #else
        // assume single int on A only
        case 6:
        case 9:
            ticks_+=2;
            break;
        case 3:
        case 12:
            ticks_-=2;
            break;
        #endif

        // no change
        case 0:
        case 5:
        case 10:
        case 15:
        default:
            errors_ |= _BV(ENCODER_ERROR_EXTRA_TICK);
            return;
            break;
    }

    // non-error ticks need to save state for next cycle
    pin_state_last_ = pin_state;
    return;
}


long Encoder::GetTicks() {
    cli();
    long tick_to_return = ticks_;
    sei();
    return tick_to_return;
}


uint8_t Encoder::GetErrors() {
    cli();
    uint8_t errors_to_return = errors_;
    sei();
    return errors_to_return;
}


void Encoder::Reset() {
    cli();
    ticks_ = 0;
    sei();
}


void Encoder::PrintDebugInfo(FILE *out_stream) {
    fprintf(out_stream, "last: %x\r\n", pin_state_last_);
    return;
}
