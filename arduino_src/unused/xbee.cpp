/**
 * Code relating to xbee
 * 
 * This has been deprecated in favor of receiver.h
 */
#include "Arduino.h"

static int xbee_brake;
static int xbee_angle;
static int xbee_connected;

String message;
char intbuf[32];

int pingPong = 1;
int startfound = 0;
int midfound = 0;
int endfound = 1;

void xbee_init() {


}

unsigned long timer = 0;
byte data;

void xbee_loop() {
  // receive and parse message from xbee
  if(Serial1.available() > 0) {

    timer = millis();

    // read message from xbee
    data = Serial1.read();

    // parse message data
    if (data == 'A') {
      startfound = 1;
      midfound = 0;
      endfound = 0;
      message = "";
    }
    else if (data == 'B') {
      startfound = 0;
      midfound = 1;
      endfound = 0;
    }
    else if (data == 'C') {
      startfound = 0;
      midfound = 0;
      endfound = 1;
    }
    else if (startfound) {
      message = message + data;
    }
    else if (midfound) {
      if(data == '1')
        brake = 1;
      else
        brake = 0;
      message.toCharArray(intbuf, sizeof(intbuf));
      steeringAngle = atoi(intbuf);
    }

    // flop external LED everytime message is recieved
    if ( pingPong == 0 ) {
      digitalWrite(4, LOW);
    }
    else {
      digitalWrite(4, HIGH);
    }

    pingPong = 1 - pingPong;

  } // end receive message
}
