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

#ifndef LIB_TYPES_H
#define LIB_TYPES_H
#ifdef __cplusplus
extern "C"{
#endif

#include <stdint.h>
#include <Arduino.h>

#define HEAD 0xFC
#define ONE_BYTE_SIZE 8
#define TWO_BYTE_SIZE 16

typedef enum 
{
	RET_OK,
	RET_ERROR
}Ret_E;


#ifdef __cplusplus
}
#endif

#endif // LIB_TYPES_H
