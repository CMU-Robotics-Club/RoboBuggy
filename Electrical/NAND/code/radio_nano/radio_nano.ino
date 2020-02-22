#include <Wire.h>

#define SLAVE_ADDRESS 0x08
#define RADIO_MESSAGE_LEN 25

unsigned char radio_data [RADIO_MESSAGE_LEN];
uint16_t send_data;
int msg_index;
uint8_t prev_msg;

void setup() {
  // open serial connection with the radio
  Serial.begin(100000, SERIAL_8E2);

  // open i2c connection to raspberry pi
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(send_radio_data_to_pi);

  send_data = 0x0000;
  msg_index = 0;
  prev_msg = 0x00;
}

void format_data() {
  // if reached the end of a valid message
  // brake data is in 5th multiple of 11 bits in message
  
  uint16_t brake_data = 
    (radio_data[6] >> 4) | 
    ((radio_data[7] & 0x7F) << 4);
  uint16_t brake = brake_data == 0x00ac; // when brakes deployed
  uint16_t auton_data = 
    (radio_data[7] >> 7) | 
    (radio_data[8] << 1) | 
    ((radio_data[9] & 0x3) << 9);
  uint16_t auton = auton_data == 0x00ac;
  uint16_t steering_data = 0;
  steering_data = 
    (radio_data[2] >> 3) | 
    ((radio_data[3] & 0x3F) << 5);
  uint16_t steering_angle = map(steering_data, 0x00ac, 0x0713, 0, 255); //2047 - steering_data; // since steering is negative

  // message we send
  send_data = steering_angle | (1 << 15) | (auton << 14) | (brake << 13); 
}

void serialEvent() {
  uint8_t msg = Serial.read();
  int is_radio_connected = 0;

  if (prev_msg == 0x00 && msg == 0x0F) {
    msg_index = 0;
  }
  
  prev_msg = msg;
  radio_data[msg_index] = msg;
  msg_index += 1;

  if (
    msg_index == RADIO_MESSAGE_LEN &&
    radio_data[0] == 0x0F && 
    radio_data[RADIO_MESSAGE_LEN - 1] == 0x00 &&
    radio_data[RADIO_MESSAGE_LEN - 2] == 0x00
  ) {
    // radio_data[RADIO_MESSAGE_LEN - 2] is 0 when no frames lose, 
    // 4 when frames are lost, and c when there is no connection
    format_data();
  }
  
  msg_index = msg_index % 25;
}

void loop() {
  
}

void send_radio_data_to_pi() {
  // sends send_data over i2c
//  char i2c_message [2];
////  i2c_message[0] = (send_data & 0xff00) >> 8;
////  i2c_message[1] = send_data & 0xff;  
//  i2c_message[0] = 0x01;
//  i2c_message[1] = 0x02;
//  Wire.write(i2c_message, 2);
////  Wire.write(radio_data, RADIO_MESSAGE_LEN);
//  send_data = 0x0000;
  byte test[2];
  test[0] = 1;
  test[1] = 2;
  Wire.write(test, 2);
}
