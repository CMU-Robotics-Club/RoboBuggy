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

#include "rbserialmessages.h"
#include "lib_encoder.h"


static unsigned long g_last_loop_time;
static unsigned long g_this_loop_time;
static unsigned long g_last_enc_count;
static unsigned long g_this_enc_count;
RBSerialMessages g_rbserialmessages;


void setup()
{
  g_rbserialmessages.Begin(&Serial);
  encoder_init();
  encoder_reset();
  g_last_loop_time = 0;
  g_this_loop_time = 0;
  g_last_enc_count = 0;
  g_this_enc_count = 0;
}


void loop()
{
  // send identity header
  g_rbserialmessages.SendSingle(RBSM_MID_RESERVED, RBSM_DID_DRIVE_ENCODER);

  // get data for this loop
  g_this_loop_time = millis();
  g_this_enc_count = encoder_get_count();

  // send new data
  g_rbserialmessages.SendDouble(RBSM_MID_ENC_TICKS_LAST_H,
                                RBSM_MID_ENC_TICKS_LAST_L,
                                g_this_enc_count - g_last_enc_count);
  g_rbserialmessages.SendDouble(RBSM_MID_ENC_TICKS_RESET_H,
                                RBSM_MID_ENC_TICKS_RESET_L,
                                g_this_enc_count);
  g_rbserialmessages.SendDouble(RBSM_MID_ENC_TIMESTAMP_H,
                                RBSM_MID_ENC_TIMESTAMP_L,
                                g_this_loop_time);

  // set carry over values
  g_last_loop_time = g_this_loop_time;
  g_last_enc_count = g_this_enc_count;

  // spin wait until next time
  while(millis() - g_this_loop_time < LOOP_TIME) {}
}
