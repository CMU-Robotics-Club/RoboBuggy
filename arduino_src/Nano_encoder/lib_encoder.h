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
  void enc_init();
	
  //returns the count of the encoder
  unsigned long get_enc_count();

  //resets the encoder count to 0
  void reset_enc();  

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* _ENCODER_H_ */
