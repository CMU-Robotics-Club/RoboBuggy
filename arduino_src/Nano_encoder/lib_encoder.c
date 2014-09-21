/**
 * @file lib_encoder.c
 * @author Joseph Paetz (rpaetz)
 */
#include <Arduino.h>
#include "lib_encoder.h"
#include "lib_encoder_def.h"

//-------local Variables-----------------
//required to be volatile for interrupts
volatile unsigned long encCount = 0;

//-------local function prototypes-----------
void increment_encoder();

//------------global functions---------------
//sets up the encoder with the given Pin
void enc_init(){
  pinMode(ENC_PIN, INPUT);
  attachInterrupt(INTERRUPT_NUM, increment_encoder, RISING); 
}

unsigned long get_enc_count(){
  return encCount;
}

void reset_enc(){
  encCount = 0; 
}

//-----------local Functions------------------
//this will count every time the encoder goes from 
//high to low or low to high
void increment_encoder(){
	encCount++;
}
