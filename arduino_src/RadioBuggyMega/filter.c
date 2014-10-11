/**
 * @file filter.h
 * @author Zach Dawson (zsd)
 *
 * This is an averageing filter. Computes the average of 
 * the last FILTER_LEN inputs.
 */

#include <Arduino.h>
#include "receiver.h"
#include "filter.h"

// This only does preliminary initialization.
// More initialization is done in the function body.
void filter_init(struct filter_state *state) {
    state->started = 0;
}

int filter_loop(struct filter_state *state, int val){
     int i;
    // First initialization: on assumption val is never -1
    if(!(state->started)){
    // On Initialization, assume our "first" reading
    //    occurred N times, and is our history
        for(i = 0; i < FILTER_LEN; i++){
            state->prev_values[i] = val;
        }
        state->pos = 0; // initialize the positions of the next number to be changed
        state->started = 1;
        return val;
    } 
    
    // NOT first initialization. 
    state->prev_values[state->pos] = val;  // replace the number in the array
    state->pos = (state->pos+1)%FILTER_LEN;// increment the pos number
    // calculate average of array
    int sum = 0;
    for(i = 0; i < FILTER_LEN; i++){
        sum += state->prev_values[i];
    }
    
    // Return average of filter
    int avg = sum/FILTER_LEN;
    return avg;

}
