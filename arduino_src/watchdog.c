/* Watchdog Library. Woof! woof!
 * 	Cats beware! Meow.
 * 
 * The watchdog library checks everytime its called how long time has passed
 * between change in signals. If the time passed between signals is more than
 * the threshold initialized in the begining, it enters the fail case. In the case
 * of buggy, it drops the brakes. 
 * 
 * The watchdog can be fed, preferably with a bone, everytime a signal is made. 
 * ie. rc_available is true. If the dog has been fed recently, then the time last fed
 * is reset to the current time. 
 *
 * @author: Audrey Yeoh (ayeoh)
 * 
 */
 
#include <Arduino.h>
#include "watchdog.h" 

static fail_function_ptr fail_mode = NULL;
static unsigned long time;
static int thresh;

/* Initializes the watchdog.
 * Sets the threshold of the maximum time to wait for a new signal
 * Sets the fail case function if the time passed between signals is more
 * than the threshold. 
 */
void watchdog_init(int timeThresh, fail_function_ptr f ){
	thresh = timeThresh;
	fail_mode = f;
	time = 0;
}

void watchdog_feed(){
	time = millis();
}

void watchdog_loop(){
	if(millis() - time > thresh){
		fail_mode();
	}
}
