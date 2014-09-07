/********************************************************************************
 * All rights are open to the public, to use, share, dump, spread for the       *
 * foreseeable future. This software and documentation constitutes purely 		*
 * published and public work, and contains open source knowledge by a bunch		*
 * of college kids who just want to have fun. All the material and code may be 	*
 * used, copied, duplicated, changed, disclosed... with any human's free will.	*
 * Have a nice day! :)															*
 ********************************************************************************/

/* @author: Audrey Yeoh (ayeohmy@gmail.com)
 * @date: 7/22/2014
 */

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "lib_types.h"


/********************************************************************************
*                     			 D E F I N I T I O N S
********************************************************************************/
#define HEAD 0xFC
#define ONE_BYTE_SIZE 8
#define TWO_BYTE_SIZE 16



/********************************************************************************
*                      L O C A L    P R O T O T Y P E S
********************************************************************************/
static uint32_t create_msg( uint8_t id, uint16_t msg);
static Ret_E send_msg( uint32_t message );	// TODO: need to implement actual sending of message

/********************************************************************************
*                       G L O B A L    F U N C T I O N S
********************************************************************************/
/* This will set up the head, and append all the messages ready to be sent and 
 * prepare the message
 * And then send the message with a return state being ok if successful 
 */
Ret_E protocol_send( uint8_t id, uint16_t msg )
{
	Ret_E success = RET_ERROR;
	// create the message
	uint32_t message = create_msg(id, msg);
	// if more messages, append? << What do?

	// send the message
	success = send_msg( message );
	// return state
	return success;

}


/********************************************************************************
*                       L O C A L    F U N C T I O N S
********************************************************************************/

static uint32_t create_msg( uint8_t id, uint16_t msg)
{
	uint8_t message = HEAD;
	message = message << ONE_BYTE_SIZE;
	message = message | id;
	message = message << TWO_BYTE_SIZE;
	message = message | msg;
	return message;
}

static Ret_E send_msg( uint32_t message )
{
	Ret_E success = RET_ERROR;
	// TODO!!!!! Send message

	return success;
}