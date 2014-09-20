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

#include "lib_protocol.h"


/********************************************************************************
*                     			 D E F I N I T I O N S
********************************************************************************/



/********************************************************************************
*                      L O C A L    P R O T O T Y P E S
********************************************************************************/
//static unsigned long create_msg( byte id, unsigned int msg);
static int send_msg( unsigned long message );	// TODO: need to implement actual sending of message

/********************************************************************************
*                       G L O B A L    F U N C T I O N S
********************************************************************************/
/* This will set up the head, and append all the messages ready to be sent and 
 * prepare the message
 * And then send the message with a return state being ok if successful 
 */
unsigned long protocol_send( byte id, unsigned int msg )
{
//	Ret_E success = RET_ERROR;
	// create the message
//	unsigned long message = create_msg(id, msg);

        unsigned long message = HEAD;
        message = message << ONE_BYTE_SIZE;
        message = message | id;
        message = message << TWO_BYTE_SIZE;
        message = message | msg;
        
	// if more messages, append? << What do?

	// send the message
	//success = send_msg( message );
	// return state        

	return message;

}


/********************************************************************************
*                       L O C A L    F U N C T I O N S
********************************************************************************/

static unsigned long create_msg( byte id, unsigned int msg)
{
	unsigned long message = HEAD;
	message = message << ONE_BYTE_SIZE;
	message = message | id;
	message = message << TWO_BYTE_SIZE;
	message = message | msg;
	return message;
}

static int send_msg( unsigned long message )
{
	int success = -1;
	// TODO!!!!! Send message

	return success;
}
