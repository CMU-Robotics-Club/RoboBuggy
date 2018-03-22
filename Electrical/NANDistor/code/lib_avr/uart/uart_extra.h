#ifndef UART_EXTRA_H
#define UART_EXTRA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "uart.h"

/************************************************************************
Title:    UART Library Extentions for Files/Streams
Author:   Ian Hartwig

LICENSE:
  Copyright (C) 2015 Ian Hartwig

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

Note:
  This library uses Peter Fleury and Andy Gock's UART Library. See their
  included license files for additional license terms.

Note:
  Use the following compiler flags to selectively compile in UART functions:
  -DUSART0_ENABLED
  -DUSART1_ENABLED
  -DUSART2_ENABLED
  -DUSART3_ENABLED
    
************************************************************************/


#define UART_GET_ERROR(b) ((b >> 8) & 0xFF)
#define UART_GET_DATA(b) (b & 0xFF)


/** @brief Extend avr libc's FILE struct with UART specific functions
 *
 *  We use this particularly to implement non-blocking fgetc with available() +
 *  avr libc fgetc().
 */
typedef struct __uartfile : __file {
  uint16_t (*available)(void);
} UARTFILE;


#if defined(USART0_ENABLED) /* Assume we will never enable UART that doesn't exist. */

/** @brief  Match uart0_getc to avr-libc getc() type. */
inline int uart0_getc_stream(FILE *stream) {
  uint16_t new_data;
  do {
    new_data = uart0_getc();
  } while(UART_GET_ERROR(new_data) != 0);
  return UART_GET_DATA(new_data);
}

/** @brief  Match uart0_putc to avr-libc putc() type. */
inline int uart0_putc_stream(char data, FILE *stream) {
  uart0_putc(data);
  return 0;
}

/**
 * @brief   Configure a provided file struct to use uart0. Designed to be used
 *          in place of the fdevopen() call from avr-libc. More info:
 *          http://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
 * @param   stream FILE struct to set up with uart0 get/put functions.
 * @return  none
 */
inline void uart0_fdevopen(UARTFILE *stream) {
  stream->get = uart0_getc_stream;
  stream->put = uart0_putc_stream;
  stream->flags = _FDEV_SETUP_RW;
  stream->available = uart0_available;
}

#endif


#if defined(USART1_ENABLED)

/** @brief  Match uart1_getc to avr-libc getc() type. */
inline int uart1_getc_stream(FILE *stream) {
  uint16_t new_data;
  do {
    new_data = uart1_getc();
  } while(UART_GET_ERROR(new_data) != 0);
  return UART_GET_DATA(new_data);
}

/** @brief  Match uart1_putc to avr-libc putc() type. */
inline int uart1_putc_stream(char data, FILE *stream) {
  uart1_putc(data);
  return 0;
}

/** @brief  Configure a provided file struct to use uart1. */
inline void uart1_fdevopen(UARTFILE *stream) {
  stream->get = uart1_getc_stream;
  stream->put = uart1_putc_stream;
  stream->flags = _FDEV_SETUP_RW;
  stream->available = uart1_available;
}

#endif


#if defined(USART2_ENABLED)

/** @brief  Match uart2_getc to avr-libc getc() type. */
inline int uart2_getc_stream(FILE *stream) {
  uint16_t new_data;
  do {
    new_data = uart2_getc();
  } while(UART_GET_ERROR(new_data) != 0);
  return UART_GET_DATA(new_data);
}

/** @brief  Match uart2_putc to avr-libc putc() type. */
inline int uart2_putc_stream(char data, FILE *stream) {
  uart2_putc(data);
  return 0;
}

/** @brief  Configure a provided file struct to use uart2. */
inline void uart2_fdevopen(UARTFILE *stream) {
  stream->get = uart2_getc_stream;
  stream->put = uart2_putc_stream;
  stream->flags = _FDEV_SETUP_RW;
  stream->available = uart2_available;
}

#endif


#if defined(USART3_ENABLED)

/** @brief  Match uart3_getc to avr-libc getc() type. */
inline int uart3_getc_stream(FILE *stream) {
  uint16_t new_data;
  do {
    new_data = uart3_getc();
  } while(UART_GET_ERROR(new_data) != 0);
  return UART_GET_DATA(new_data);
}

/** @brief  Match uart3_putc to avr-libc putc() type. */
inline int uart3_putc_stream(char data, FILE *stream) {
  uart3_putc(data);
  return 0;
}

/** @brief  Configure a provided file struct to use uart3. */
inline void uart3_fdevopen(UARTFILE *stream) {
  stream->get = uart3_getc_stream;
  stream->put = uart3_putc_stream;
  stream->flags = _FDEV_SETUP_RW;
  stream->available = uart3_available;
}

#endif


#ifdef __cplusplus
}
#endif

#endif // UART_EXTRA_H 
