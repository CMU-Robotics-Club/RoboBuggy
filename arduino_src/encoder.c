/**
 * @file encoder.c
 * @author Audrey Yeoh (ayeoh)
 * @author Matt Sebek (msebek0
 */
#include <Arduino.h>

static int input_pin;

static int count=0;
// 1 if was high, 0 if was low
static int state=0;
// ?? 
static int last_time;
static int print_period = 5;

void encoder_init(int encoder_pin) {
  input_pin = encoder_pin;
  pinMode(input_pin, 1);
}

// Every X ms, publish. (or perish!!)
void encoder_publish() {
  if (last_time < (millis() - print_period)) {
    last_time = millis();
  }
}

int encoder_get_count() {
  return count;
}

// Lightweight, checks low-pri encoder loop
void encoder_loop(){
  if (state != digitalRead(input_pin)){
    count++;
    state = !state;
  }
}
