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

bool RBSerialMessages::Available(){
    serial_stream_ = serial_stream;
    serial_stream_->available(RBSM_BAUD_RATE);

    return serial_stream_;
}

rb_message_t* RBSerialMessages::Read(rb_message_t* ptr){

    char footer = serial_stream_->read(RBSM_BAUD_RATE);
    while (footer != RBSM_FOOTER){    // read until found the footer
        footer = serial_stream_->read(RBSM_BAUD_RATE);  // null segfault?
    }


    char message = serial_stream_->read(RBSM_BAUD_RATE);
    ptr->message_id = message;
    char message1 = serial_stream_->read(RBSM_BAUD_RATE);
    char message2 = serial_stream_->read(RBSM_BAUD_RATE);
    char message3 = serial_stream_->read(RBSM_BAUD_RATE);
    char message4 = serial_stream_->read(RBSM_BAUD_RATE);
    uint32_t data = (message1<<RBSM_THREE_BYTE_SIZE) | (message2<<RBSM_TWO_BYTE_SIZE) | (message3<<RBSM_ONE_BYTE_SIZE) | message4;
    ptr->data = data;
    message = serial_stream_->read(RBSM_BAUD_RATE);
    ptr->footer_id = message;

    // maybe check if the footer matches buffer_in_, else throw the message cause it is sad.


    return ptr;
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
