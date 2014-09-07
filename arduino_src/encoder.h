/**
 * @file encoder.h
 * @author Audrey Yeoh (ayeoh)
 * @author Matt Sebek (msebek0
 */

void encoder_init(int encoder_pin);

// Every X ms, publish.
void encoder_publish();

int encoder_get_count();

// Lightweight, checks low-pri encoder loop
void encoder_loop();
