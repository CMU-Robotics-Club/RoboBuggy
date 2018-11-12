/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#define RECEIVING 1
#define STEERING_PIN 0
#define BRAKE_PIN 4
#define AUTON_PIN 3
#define BRAKE_OFFSET 10
#define AUTON_OFFSET 11

RF24 radio(7,8); // CE, CSN
const byte address[6] = "00001";
#if RECEIVING
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  if (radio.available()) {
    int text[32];
    radio.read(&text, sizeof(text));
    Serial.print(text[0]);
    Serial.print(" ");
    Serial.println(text[1]);
  }
}
#else
int message = 0;
void setup() {
  Serial.begin(57600);
  pinMode(STEERING_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT); // maybe make next two pullups and take out the resistors
  pinMode(AUTON_PIN, INPUT);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}
void loop() {
  message = analogRead(STEERING_PIN);
  message += ((digitalRead(BRAKE_PIN) == HIGH) ? 1 :0) << BRAKE_OFFSET;
  message += ((digitalRead(AUTON_PIN) == HIGH) ? 1 :0) << AUTON_OFFSET;
  radio.write(&message, sizeof(int));
  Serial.println(message);
  message = 0;
  delay(20);
}
#endif
