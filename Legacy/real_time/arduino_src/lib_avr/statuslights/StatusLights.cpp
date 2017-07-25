#include "StatusLights.h"


/** @brief Sets up the appropriate pins for the indicator lights
 *
 */
StatusLights::StatusLights(uint8_t reg_offset, volatile uint8_t *port, volatile uint8_t *ddr) {
    light_port = port;
    register_offset = reg_offset;
    *light_port &= ~_BV(reg_offset);
    *ddr |= _BV(register_offset);
}

/** @brief Turns on the light
 *
 */
void StatusLights::Enable(void) {
    *light_port |= _BV(register_offset);
}

/** @brief Turns off the light
 *
 */
void StatusLights::Disable(void) {
    *light_port &= ~_BV(register_offset);
}

void StatusLights::Flash(uint8_t time) {
    //stubbed out
}

void StatusLights::FlashBlocking(uint8_t time) {
    //stubbed out
}