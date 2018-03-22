#include <Arduino.h>
#include "receiver.h"
#include "filter.h"

// This only does preliminary initialization.
// More initialization is done in the function body.
void filter_init(struct filter_state *state) {
    state->started = 0;
}

//<<<<<<< Updated upstream
int filter_loop(struct filter_state *state, int val){
     int i;
    // First initialization: on assumption val is never -1
    if(!(state->started)){
        for(i = 0; i < FILTER_LEN; i++){
            state->prev_values[i] = val;
//=======
//#define SIZE 5 // the array size
//static char started = 0;
//static int last_val_sent;
//static int values[SIZE];
//static int pos;
//
///**
// * @brief Filters noise from raw RC readings
// *
// */
//int filter(int new_raw_reading){
//     int i;
//   
//    // On Initialization, assume our "first" reading
//    //    occurred N times, and is our history
//    if(!started){
//        for(i = 0; i < SIZE; i++){
//            values[i] = new_raw_reading;
//>>>>>>> Stashed changes
        }
        state->pos = 0; // initialize the positions of the next number to be changed
        state->started = 1;
        return val;
    } 
    
    
//<<<<<<< Updated upstream
    // NOT first initialization. 
    state->prev_values[state->pos] = val;  // replace the number in the array
    state->pos = (state->pos+1)%FILTER_LEN;// increment the pos number
    // calculate average of array
//=======
//    
//        
//    values[pos] = val;  // replace the number in the array
//    pos = (pos+1)%SIZE; // increment the pos number
//    
//    // calculate average of previous readings
//>>>>>>> Stashed changes
    int sum = 0;
    for(i = 0; i < FILTER_LEN; i++){
        sum += state->prev_values[i];
    }
//<<<<<<< Updated upstream
    int avg = sum/FILTER_LEN;
    // return.
    return avg;
//=======
//    // Return the average.
//    return sum / SIZE;
//>>>>>>> Stashed changes

}
