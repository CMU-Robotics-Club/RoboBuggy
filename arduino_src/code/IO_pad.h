/********************************************************************************
 * All rights are open to the public, to use, share, dump, spread for the       *
 * foreseeable future. This software and documentation constitutes purely 		*
 * published and public work, and contains open source knowledge by a bunch		*
 * of college kids who just want to have fun. All the material and code may be 	*
 * used, copied, duplicated, changed, disclosed... with any human's free will.	*
 * Have a nice day! :)															*
 ********************************************************************************/

#ifndef IO_PAD_H_
#define IO_PAD_H_

#include "IO_types.h"


IO_ret_e IO_set_pad( uint8_t pad, uint8_t value);
IO_ret_e IO_set_state( uint8_t pad, IO_state_e state );


#endif