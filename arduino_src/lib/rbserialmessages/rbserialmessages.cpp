/********************************************************************************
 * All rights are open to the public, to use, share, dump, spread for the       *
 * foreseeable future. This software and documentation constitutes purely 		*
 * published and public work, and contains open source knowledge by a bunch		*
 * of college kids who just want to have fun. All the material and code may be 	*
 * used, copied, duplicated, changed, disclosed... with any human's free will.	*
 * Have a nice day! :)															*
 ********************************************************************************/

/**
 * @author: Ian Hartwig (spacepenguine@gmail.com)
 * @author: Audrey Yeoh (ayeohmy@gmail.com)
 * @date: 10/3/2014
 */

#include "rbserialmessages.h"


// Public //////////////////////////////////////////////////////////////////////

RBSerialMessages::RBSerialMessages() {
}


int RBSerialMessages::Begin(HardwareSerial *serial_stream) {
  // setup the hardware serial
  serial_stream_ = serial_stream;
  serial_stream_->begin(RBSM_BAUD_RATE);

  // success
  return 1;
}

int RBSerialMessages::Send(uint8_t id, uint32_t message) {
  uint8_t buffer_pos;
  buffer_pos = InitMessageBuffer();
  buffer_pos = AppendMessageToBuffer(id, message, buffer_pos);
  
  serial_stream_->write(buffer_out_, buffer_pos);

  // success
  return 0;
}

// Private /////////////////////////////////////////////////////////////////////
uint8_t RBSerialMessages::AppendMessageToBuffer(uint8_t id,
                                                uint32_t message,
                                                uint8_t out_start_pos) {
  uint8_t buffer_pos = out_start_pos;
  uint8_t message_ll = message & RBSM_ONE_BYTE_MASK;
  uint8_t message_lh = (message >> RBSM_ONE_BYTE_SIZE) & RBSM_ONE_BYTE_MASK;
  uint8_t message_hl = (message >> RBSM_TWO_BYTE_SIZE) & RBSM_ONE_BYTE_MASK;
  uint8_t message_hh = (message >> RBSM_THREE_BYTE_SIZE) & RBSM_ONE_BYTE_MASK;

  // write message id (and id since single message)
  buffer_out_[buffer_pos++] = id;

  buffer_out_[buffer_pos++] = message_hh;
  buffer_out_[buffer_pos++] = message_hl;
  buffer_out_[buffer_pos++] = message_lh;
  buffer_out_[buffer_pos++] = message_ll;

  buffer_out_[buffer_pos++] = RBSM_FOOTER;

  // write null terminator just in case. note no increment
  buffer_out_[buffer_pos] = RBSM_NULL_TERM;


  return buffer_pos;
}

uint8_t RBSerialMessages::InitMessageBuffer() {
  uint8_t buffer_pos = 0;

  // write null terminator just in case. note no increment
  buffer_out_[buffer_pos] = RBSM_NULL_TERM;

  return buffer_pos;
}
