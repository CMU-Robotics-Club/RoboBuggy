/**
 * @file lib_encoder.h
 * @author Joseph Paetz (rpaetz)
 */
#ifndef _LIB_ENCODER_H_
#define _LIB_ENCODER_H_

#define ENC_PIN 2
//this is the interrupt connected to pin 2
#define INTERRUPT_NUM 0


#ifdef __cplusplus
extern "C"{
#endif
  //sets up the encoder with the given Pin
  void encoder_init();
	
  //returns the count of the encoder
  unsigned long encoder_get_count();

  //resets the encoder count to 0
  void encoder_reset();  

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* _ENCODER_H_ */
