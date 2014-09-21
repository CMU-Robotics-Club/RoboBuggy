/********************************************************************************
 * All rights are open to the public, to use, share, dump, spread for the       *
 * foreseeable future. This software and documentation constitutes purely 		*
 * published and public work, and contains open source knowledge by a bunch		*
 * of college kids who just want to have fun. All the material and code may be 	*
 * used, copied, duplicated, changed, disclosed... with any human's free will.	*
 * Have a nice day! :)															*
 ********************************************************************************/

/* @author: Audrey Yeoh (ayeohmy@gmail.com)
 * @date: 7/22/2014
 */

#include "rbserialmessages.h"


// Public //////////////////////////////////////////////////////////////////////

RBSerialMessages::RBSerialMessages() {
}


int RBSerialMessages::Begin(HardwareSerial *serial_stream) {
  // setup the hardware serial
  serial_stream_ = serial_stream;
  serial_stream_->begin(RBSM_BAUD_RATE);

  // send the device id message
  SendSingle(RBSM_MID_DEVICE_ID, RBSM_DID_DRIVE_ENCODER);

  // success
  return 1;
}


int RBSerialMessages::SendSingle(uint8_t id, uint16_t message) {
  uint8_t buffer_pos;
  buffer_pos = InitMessageBuffer();
  buffer_pos = AppendMessageToBuffer(id, message, buffer_pos);
  
  serial_stream_->write(buffer_out_, buffer_pos);

  // success
  return 0;
}

// Private /////////////////////////////////////////////////////////////////////
uint8_t RBSerialMessages::AppendMessageToBuffer(uint8_t id,
                                                uint16_t message,
                                                uint8_t out_start_pos) {
  uint8_t buffer_pos = out_start_pos;
  uint8_t message_l = message & RBSM_BYTE_MASK;
  uint8_t message_h = (message >> RBSM_ONE_BYTE_SIZE) & RBSM_BYTE_MASK;

  // write message id (and id since single message)
  buffer_out_[buffer_pos++] = RBSM_HEAD;
  buffer_out_[buffer_pos++] = id;

  // sanitize message content if necessary
  if(message_h == RBSM_HEAD) {
    buffer_out_[buffer_pos++] = message_h;
    buffer_out_[buffer_pos++] = message_h;
  } else {
    buffer_out_[buffer_pos++] = message_h;
  }

  if(message_l == RBSM_HEAD) {
    buffer_out_[buffer_pos++] = message_l;
    buffer_out_[buffer_pos++] = message_l;
  } else {
    buffer_out_[buffer_pos++] = message_l;
  }

  // write null terminator just in case. note no increment
  buffer_out_[buffer_pos] = RBSM_NULL_TERM;

  return buffer_pos;
}

uint8_t RBSerialMessages::InitMessageBuffer() {
  uint8_t buffer_pos = 0;

  // message header
  buffer_out_[buffer_pos++] = 0;

  // write null terminator just in case. note no increment
  buffer_out_[buffer_pos] = RBSM_NULL_TERM;

  return buffer_pos;
}
