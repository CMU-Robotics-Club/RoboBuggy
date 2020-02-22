#include <Wire.h>

uint8_t steering_angle;
bool autonomous_mode;
bool brakes;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  steering_angle = 0;
  autonomous_mode = false;
  brakes = true;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("a");
  Wire.requestFrom(0x08, 1);
  Serial.println("b");

  byte radio_msg[2];
  radio_msg[0] = 0;
  radio_msg[1] = 0;

  int i = 0;
  Serial.println("here");
  while(Wire.available())
  {
//    Serial.println("here");
//    radio_msg[i] =/ Wire.read();
    Serial.println(Wire.read());
    i++;
  }

//  if (radio_msg[0] != 0) {
//    steering_angle = (uint8_t)radio_msg[1];
//  } else {
//    brakes = true;
//  }
//
//  Serial.println(radio_msg[1]);
  delay(1000);
}
