/*
 *  Modified from 
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code            
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
// 0 would be for transmitter, 1 for receiver
#define LOW_VOLTAGE 650 // after a month of no charge battery at 645
#define RECEIVING 0
#define STEERING_PIN 0
#define BRAKE_PIN 4
#define AUTON_PIN 3
#define BRAKE_OFFSET 10
#define AUTON_OFFSET 11
#define voltage_test A1
#define low_volt_light 5
bool did_connect = 1;

RF24 radio(7,8); // CE, CSN
const byte address[6] = "00001";
#if RECEIVING
void setup() {
  Serial.begin(9600);
  if(radio.begin()==false)
  {
    Serial.print("didn't connect");
    did_connect = 0;
  }
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void loop() {
  if(!did_connect)
  {
    Serial.println("didn't connect");
  }
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
  pinMode(BRAKE_PIN, INPUT_PULLUP); // maybe make next two pullups and take out the resistors
  pinMode(AUTON_PIN, INPUT_PULLUP);
  if(radio.begin()==false)
  {
    Serial.print("didn't connect");
    did_connect = 0;
  }
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  pinMode(voltage_test, INPUT);
  pinMode(low_volt_light, OUTPUT);
}
void loop() {
  message = analogRead(STEERING_PIN);
  message += ((digitalRead(BRAKE_PIN) == HIGH) ? 1 :0) << BRAKE_OFFSET;
  message += ((digitalRead(AUTON_PIN) == HIGH) ? 1 :0) << AUTON_OFFSET;
  if(!did_connect)
  {
    Serial.println("didn't connect");
  }
  else
  {
    radio.write(&message, sizeof(int));
  }
  Serial.println(analogRead(voltage_test));
  if(analogRead(voltage_test)<LOW_VOLTAGE)
  {
    digitalWrite(low_volt_light, HIGH);
  }
  else
  {
    digitalWrite(low_volt_light, LOW);
  }
  Serial.println(message);
  message = 0;
  delay(10);
}
#endif
