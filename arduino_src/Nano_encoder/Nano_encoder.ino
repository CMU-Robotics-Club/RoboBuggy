/********************************************************************************
 * All rights are open to the public, to use, share, dump, spread for the       *
 * foreseeable future. This software and documentation constitutes purely 		*
 * published and public work, and contains open source knowledge by a bunch		*
 * of college kids who just want to have fun. All the material and code may be 	*
 * used, copied, duplicated, changed, disclosed... with any human's free will.	*
 * Have a nice day! :)															*
 ********************************************************************************/

/* This program spits out encoder data every 100 ms.
 * @author: Audrey Yeoh (ayeoh) 
 */

#define LOOP_TIME 10

#include "lib_protocol.h"
#include "lib_types.h"
#include "lib_encoder.h"

#define DEBUG
#ifdef DEBUG
#  define dbg_Serial_print(...) Serial.println(__VA_ARGS__)
#else
#  define dbg_Serial_print(...)
#endif

unsigned long loop_starttime = 0;
unsigned long last_looptime = 0;
unsigned long reset_count;
unsigned long last_count;
unsigned long current_enc_val;

void setup()
{
  Serial.begin(9600);
  enc_init();
  reset_count = 0; // this is for the reset reset_count
  last_count = 0;
  current_enc_val = 0;
}


void loop()
{
  /*************** HEAD OF TIMING LOOP **************/
  loop_starttime = millis();
  /*************** END OF TIMING LOOP HEAD **********/
  
  current_enc_val = get_enc_count();
  // D CODE GOES HEREEEEEE
  last_count = current_enc_val - reset_count;
  // get reset_count (already stored) TODO!
  reset_count = current_enc_val;
  
  // split the last_count
  unsigned int upper_message = last_count >> 16;
  unsigned int lower_message = last_count & 0xFF;
  // send the two parts of last_count
  Serial.write(protocol_send(ENC_BYTE_ONE_TICK_LAST, upper_message));
  dbg_Serial_print(protocol_send(ENC_BYTE_ONE_TICK_LAST, upper_message), HEX);  
  Serial.write(protocol_send(ENC_BYTE_TW0_TICK_LAST, lower_message));
  dbg_Serial_print(protocol_send(ENC_BYTE_TW0_TICK_LAST, lower_message), HEX); 
  
  // split the reset_count 
  upper_message = reset_count >> 16;
  lower_message = reset_count & 0xFF;
  // send the two parts of reset_count
  Serial.write(protocol_send(ENC_BYTE_ONE_TICK_RESET, upper_message));
  dbg_Serial_print(protocol_send(ENC_BYTE_ONE_TICK_RESET, upper_message), HEX);
  Serial.write(protocol_send(ENC_BYTE_TWO_TICK_RESET, lower_message));
  dbg_Serial_print(protocol_send(ENC_BYTE_TWO_TICK_RESET, lower_message), HEX);
  
  dbg_Serial_print(); 

  // reset the last count after the send
  last_count = 0;
  
  /*************** TAIL OF TIMING LOOP **************/
  last_looptime = millis() - loop_starttime;
  while (last_looptime < LOOP_TIME)
  {
    // reset_count d tick?
    last_looptime = millis() - loop_starttime;
  }
  /************* END OF TAIL OF TIMING LOOP *********/
}

// Send the timestamp in two packets
void send_time(unsigned long time)
{
   unsigned int upper_message = time >> 16;
   unsigned int lower_message = time & 0xFF;
   Serial.write(protocol_send(ENC_TIMESTAMP_ONE, upper_message));
   dbg_Serial_print(protocol_send(ENC_TIMESTAMP_ONE, upper_message), HEX);  
   Serial.write(protocol_send(ENC_TIMESTAMP_TWO, lower_message));
   dbg_Serial_print(protocol_send(ENC_TIMESTAMP_TWO, lower_message), HEX);  
}
