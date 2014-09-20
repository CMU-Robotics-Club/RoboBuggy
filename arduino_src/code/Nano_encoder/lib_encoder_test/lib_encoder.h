/**
 * @file lib_encoder.h
 * @author Joseph Paetz (rpaetz)
 */
#ifndef _LIB_ENCODER_H_
#define _LIB_ENCODER_H_

#ifdef __cplusplus
extern "C"{
#endif
	//sets up the encoder with the given Pin
	void encoder_init();
	
	//returns the count of the encoder
	int get_enc_count();

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* _ENCODER_H_ */
