/********************************************************************************
 * All rights are open to the public, to use, share, dump, spread for the       *
 * foreseeable future. This software and documentation constitutes purely 		*
 * published and public work, and contains open source knowledge by a bunch		*
 * of college kids who just want to have fun. All the material and code may be 	*
 * used, copied, duplicated, changed, disclosed... with any human's free will.	*
 * Have a nice day! :)															*
 ********************************************************************************/

#ifndef LIB_PROTOCOL_H
#define LIB_PROTOCOL_H

#include "lib_types.h"


/* This will be the main structure of our packets:
 *
 * Head [0xFC] 	ID [ - ]  Message[ - -] ... ID [ - ]  Message[ - -]
 * [ 1 byte  ] 	[1 byte]  [  2 bytes  ] ... [1 byte]  [  2 bytes  ]
 */

Ret_E protocol_init( void ); // TODO: I'm not sure exactly what needs to be initialized yet
Ret_E protocol_run( void ); // TODO:
Ret_E protocol_send(uint8_t id, uint16_t message); // Currently working on



#endif // LIB_PROTOCOL_H