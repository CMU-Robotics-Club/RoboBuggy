/**
 * @file watchdog.h
 * @brief Headers for Receiver
 * 
 * @author: Audrey Yeoh (ayeoh)
 */
 
#ifndef _WATCHDOG_H_
#define _WATCHDOG_H_

#ifdef __cplusplus
extern "C"{
#endif

typedef void (*fail_function_ptr)(void);

void watchdog_init(int timeThresh, fail_function_ptr f);

void watchdog_feed();

void watchdog_loop();

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* _WATCHDOG_H_ */
