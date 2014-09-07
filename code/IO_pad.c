/********************************************************************************
 * All rights are open to the public, to use, share, dump, spread for the       *
 * foreseeable future. This software and documentation constitutes purely 		*
 * published and public work, and contains open source knowledge by a bunch		*
 * of college kids who just want to have fun. All the material and code may be 	*
 * used, copied, duplicated, changed, disclosed... with any human's free will.	*
 * Have a nice day! :)															*
 ********************************************************************************/

#include <Arduino.h>
#include "IO_pad.h"

/*********************************************************************************
 *						L O C A L    P R O T O T Y P E S						 *
 ********************************************************************************/





/*********************************************************************************
 *						G L O B A L    F U N C T I O N S						 *
 ********************************************************************************/
IO_ret_e IO_set_pad( uint8_t pad, uint8_t value )
{
	IO_ret_e success = RET_ERROR;
	analogWrite(pad, value);
	success = RET_OK;
	return success;
}


IO_ret_e IO_set_state( uint8_t pad, IO_state_e state)
{
	IO_ret_e success = RET_ERROR;
	digitalWrite(pad, state);
	success = RET_OK;
	return success;
}


/*********************************************************************************
 *							L O C A L    F U N C T I O N S						 *
 ********************************************************************************/




