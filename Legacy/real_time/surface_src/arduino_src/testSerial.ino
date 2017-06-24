#include<Servo.h>

Servo steering;
Servo brake;
byte i = 0;
byte j = 0;

void setup() {
  Serial.begin(9600);
  steering.attach(9);
  steering.write(0);
  
  brake.attach(7);
  brake.write(0);
}

boolean val = false;
int index = 0;
void loop() {
  i++;
  if (i > 5) {
    j++;
    i = 0;
  }
  
  while (Serial.available() > 0) {
    char input = (char)Serial.read();
    switch (index) {
      case 0: 
        if (input == 'T') index++;
        else if (input == 'B') index = 3;
        break;
      case 1:
        if (input == 'H') index++;
        else index = 0;
        break;
      case 2:
        steering.write((int)input);
        index = 0;
        break;
      case 3:
        if (input == 'K') index++;
        else index = 0;
        break;
      case 4:
        brake.write((int)input);
        break;
    }
  }
   
   Serial.write(0xFC);
   Serial.write(i);
   Serial.write(0);
   Serial.write(0);
   Serial.write(0);
   Serial.write(j);
   Serial.println();
   
   delay(100);
}
