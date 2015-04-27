/**
 * @file filter.h
 * @brief Filter to filter data from rc_available
 *
 * @author: Audrey Yeoh (ayeoh)
 * 
 */

#ifndef _FILTER_H_
#define _FILTER_H_

#ifdef __cplusplus
extern "C"{
#endif

//<<<<<<< Updated upstream
#define FILTER_LEN 10 // Averaging window size

struct filter_state {
    int prev_values[FILTER_LEN];
    char started;
    int pos; // next index to update
};


void filter_init(struct filter_state *state);

int filter_loop(struct filter_state *state, int val); // returns the int that has been filtered
//=======
//// @param val The next raw reading
//// @returns The next filtered reading
//int filter(int new_raw_reading); 
//          
//>>>>>>> Stashed changes


#ifdef __cplusplus
}
#endif

#endif


