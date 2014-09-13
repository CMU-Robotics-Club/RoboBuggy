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

#define LOOP_TIME 100

#include "lib_protocol.h"
#include "lib_types.h"

unsigned long loop_starttime = 0;
unsigned long last_looptime = 0;
unsigned long count = 0;

void setup()
{
  Serial.begin(9600);
}


void loop()
{
  /*************** HEAD OF TIMING LOOP **************/
  loop_starttime = millis();
  /*************** END OF TIMING LOOP HEAD **********/
  
  // D CODE GOES HEREEEEEE
  
  // get count (already stored) TODO!
  
  // split the count 
  unsigned int upper_message = count >> 16;
  unsigned int lower_message = count & 0xFF;
  // send the two counts
  protocol_send(ENC_BYTE_ONE_TICK_LAST, upper_message);
  protocol_send(ENC_BYTE_TW0_TICK_LAST, lower_message);

  
  /*************** TAIL OF TIMING LOOP **************/
  last_looptime = millis() - loop_starttime;
  while (last_looptime < LOOP_TIME)
  {
    // count d tick?
    last_looptime = millis() - loop_starttime;
  }
  /************* END OF TAIL OF TIMING LOOP *********/
}
