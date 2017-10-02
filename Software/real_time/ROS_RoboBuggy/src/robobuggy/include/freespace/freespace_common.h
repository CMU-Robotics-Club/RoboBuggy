/* * libfreespace - library for communicating with Freespace devices
 *
 * Copyright 2010-15 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef FREESPACE_COMMON_H_
#define FREESPACE_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Define the types */
#ifdef _WIN32
#include <stdint.h>
typedef void* FreespaceFileHandleType;

// All files within this DLL are compiled with the LIBFREESPACE_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// LIBFREESPACE_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef LIBFREESPACE_EXPORTS
#define LIBFREESPACE_API __declspec(dllexport)
#else
#define LIBFREESPACE_API
#endif

#else
#include <stdint.h>
typedef int FreespaceFileHandleType;
#if (defined LIBFREESPACE_EXPORTS && defined __GNUC__ && __GNUC__ >= 4)
#define LIBFREESPACE_API __attribute__ ((visibility ("default")))
#else
#define LIBFREESPACE_API
#endif  // (defined LIBFREESPACE_EXPORTS && defined __GNUC__ && __GNUC__ >= 4)
#endif

/** \ingroup initialization
 * Error codes.
 */
enum freespace_error {
    /** Success (no error) */
    FREESPACE_SUCCESS = 0,

    /** Input/output error */
    FREESPACE_ERROR_IO = -1,

    /** Access denied (insufficient permissions) */
    FREESPACE_ERROR_ACCESS = -3,

    /** No such device (it may have been disconnected) */
    FREESPACE_ERROR_NO_DEVICE = -4,

    /** Entity not found */
    FREESPACE_ERROR_NOT_FOUND = -5,

    /** Resource busy */
    FREESPACE_ERROR_BUSY = -6,

    /** Operation timed out */
    FREESPACE_ERROR_TIMEOUT = -7,

    /** Pipe error */
    FREESPACE_ERROR_PIPE = -9,

    /** System call interrupted (perhaps due to signal) */
    FREESPACE_ERROR_INTERRUPTED = -10,

    /** Out of memory */
    FREESPACE_ERROR_OUT_OF_MEMORY = -11,

    /** Amount to send was larger than the max */
    FREESPACE_ERROR_SEND_TOO_LARGE = -20,

    /** Invalid or uninitialized device handle */
    FREESPACE_ERROR_INVALID_DEVICE = -21,

    /** Receive buffer was too small */
    FREESPACE_ERROR_RECEIVE_BUFFER_TOO_SMALL = -22,

    /** Unknown error when trying to create or start a thread */
    FREESPACE_ERROR_COULD_NOT_CREATE_THREAD = -23,

    /** Buffer was too small */
    FREESPACE_ERROR_BUFFER_TOO_SMALL = -24,

    /** No data was received */
    FREESPACE_ERROR_NO_DATA = -25,

    /** No data was received */
    FREESPACE_ERROR_MALFORMED_MESSAGE = -26,

	/** Invalid HID protocol version */
	FREESPACE_ERROR_INVALID_HID_PROTOCOL_VERSION = -27,

    /** An unimplemented feature */
    FREESPACE_ERROR_UINIMPLEMENTED = -97,

    /** Any uncategorized or unplanned error */
    FREESPACE_ERROR_UNEXPECTED = -98,
};

/**
 * Address of the Freespace device to which the message will be sent.
 * These are reserved for now and must be set to 0 when calling into
 * libfreespace to send a message.
 */
typedef uint8_t FreespaceAddress;

#ifdef __cplusplus
}
#endif

#endif // FREESPACE_COMMON_H_

