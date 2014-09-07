/*
  Makes Arduino receive input from xbee and parse data.

 The circuit:
 * RX is digital pin 2 (connect to TX of XBee)
 * TX is digital pin 3 (connect to RX of XBee)

 */

//#include <SoftwareSerial.h>
#include <Servo.h>

#define BRAKE_PIN 8
#define BRAKE_INDICATOR_PIN 5

#define SERVO_PIN 9
#define STEERING_CENTER 133

#define XBEE_MSG_REC 4

Servo myservo;  // create servo object to control a servo

unsigned long timer = 0L;
char data;
String message;
char intbuf[32];
int brake = 0;
int steeringAngle = STEERING_CENTER;
int pingPong = 1;
int startfound = 0;
int midfound = 0;
int endfound = 1;

//code that keeps loop time constant each loop
int hz = 40;
int print_period = 1000 / hz;
unsigned long last_time = 0;

void setup()  {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(XBEE_MSG_REC, OUTPUT);

  // initialize the brake with brake pin and led brake pin
  brake_init(brakePin, brakeLedPin);


  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  myservo.write(STEEING_CENTER);
}

void loop() {

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

  // brake if it has been greater than 3 seconds since we last got a message
  if( (millis() - timer) > 5000L ) {
    digitalWrite(8, 0);
    digitalWrite(5, HIGH);
    exit(1);
  }

  if((millis() - timer) > 2000L) {
    digitalWrite(12, HIGH);
  }
  else {
    digitalWrite(12, LOW);
  }

  // make brake LED light up if brakes should be down
  if( brake == 0 ) {
    digitalWrite(5, HIGH);
  }
  else {
    digitalWrite(5, LOW);
  }

  // send parsed signal to brakes and servo
  digitalWrite(8, brake);

  // sets the servo position
  myservo.write(steeringAngle);
}

