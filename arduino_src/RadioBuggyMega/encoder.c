/**
 * @file encoder.c
 * @author Audrey Yeoh (ayeoh)
 * @author Matt Sebek (msebek)
 *
 * This function counts the number of encoder tics
 */
#include <Arduino.h>
#include "encoder.h"

static int enc_pin = 1;
static int count = 0; // Counts encoder 'tics'
static int state = 1; // 1 if was high (magnet nearby), 0 if was low (magnet !nearby)

void encoder_init(int encoder_pin) {
  pinMode(encoder_pin, INPUT);
  enc_pin = encoder_pin;
}

int encoder_get_count() {
  return count;
}

// Lightweight, checks low-pri encoder loop
void encoder_loop() {
  if (state != digitalRead(enc_pin)){
    count++; 
    state = !state;
  }  
}



