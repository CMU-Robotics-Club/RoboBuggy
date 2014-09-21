#include "lib_encoder.h"

int encVal = 0;

void setup() {
  enc_init();
  Serial.begin(9600); 
}

void loop() {
  encVal = get_enc_count();
  Serial.print("encoder Value: ");
  Serial.println(encVal);
  delay(100); 
}
