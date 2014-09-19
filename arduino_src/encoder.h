/**
 * @file encoder.h
 * @author Audrey Yeoh (ayeoh)
 * @author Matt Sebek (msebek)
 */
#ifndef _ENCODER_H_
#define _ENCODER_H_

#ifdef __cplusplus
extern "C"{
#endif

  void encoder_init(int encoder_pin);

  // Every X ms, publish.
  void encoder_publish();

  int encoder_get_count();

  // Lightweight, checks low-pri encoder loop
  void encoder_loop();

#ifdef __cplusplus
} // extern "C"
#endif /* __cplusplus */

#endif /* _ENCODER_H_ */