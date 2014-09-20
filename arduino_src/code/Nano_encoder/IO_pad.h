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
#include <Arduino.h>
#include "IO_types.h"
#include "lib_types.h"

#ifdef __cplusplus
extern "C"{
#endif


Ret_E IO_set_pad( byte pad, byte value);
Ret_E IO_set_state( byte pad, IO_state_e state );

#ifdef __cplusplus
}
#endif
#endif
