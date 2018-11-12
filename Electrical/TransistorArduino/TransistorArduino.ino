#include "Dynamixel_Serial.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#include <avr/wdt.h>

//Constants to do with the Dynamixel.
#define SERIAL_MODE 0
//#define FORCE_AUTONOMOUS = true; //This should only be defined for debug purposes.
#define DYNAMIXEL_BAUD 57600
#define SERVO_ID 0xFE
#define SERVO_RECEIVE_ID 0x01 //0x04
#define CW_LIMIT_ANGLE 975
#define CCW_LIMIT_ANGLE 1425

#define NEUTRAL_ANGLE 1200
#define DEFAULT_SPEED 0x100


//Define the pin numbering scheme.
#define RADIO_STEERING_PIN 2
#define RADIO_BRAKE_PIN 21
#define RADIO_AUTON_PIN 20
#define BRAKE_PIN 31
#define TRISTATE_TOGGLE_PIN 3
#define ENCODER_PIN 38
#define BATTERY_LOW_STATUS_LIGHT 42
#define RC_LOST_SIGNAL 44
#define AUTON_LOST_SIGNAL 46

//Constants to do with RBSM
#define RBSM_BAUD 57600

// Message Types
#define RBSM_MID_ENC_TICKS_RESET 1
#define RBSM_MID_ENC_RESET_REQUEST 5
#define RBSM_MID_ENC_RESET_CONFIRM 6
#define RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND 17
#define RBSM_MID_MEGA_AUTON_BRAKE_COMMAND 18
#define RBSM_MID_MEGA_STEER_COMMAND 19
#define RBSM_MID_MEGA_STEER_ANGLE 20
#define RBSM_MID_MEGA_BRAKE_STATE 21
#define RBSM_MID_MEGA_AUTON_STATE 22
#define RBSM_MID_MEGA_BATTERY_LEVEL 23
#define RBSM_MID_MEGA_STEER_FEEDBACK 24
#define RBSM_MID_COMP_HASH 30
#define RBSM_MID_MEGA_TIMESTAMP 253
#define RBSM_MID_ERROR 254
#define DEVICE_ID 255
#define FOOTER 10

// RBSM error types
#define RBSM_EID_WDT 0
#define RBSM_EID_RBSM_LOST_STREAM 1
#define RBSM_EID_RBSM_INVALID_MID 2
#define RBSM_EID_RC_LOST_SIGNAL 3
#define RBSM_EID_AUTON_LOST_SIGNAL 4

bool verboseMode = false;

#if SERIAL_MODE
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
#else

struct autonBufferElem
{
  unsigned char value;
  struct autonBufferElem* next;
};

struct autonBufferElem* first = malloc(sizeof(autonBufferElem));

struct autonBufferElem* last = first;
#endif

struct stateDescription {
  bool brakesDeployed;
  int steeringPosition; //Should be from 1950 to 2550-ish, in terms of actual ticks to send to Dynamixel.
};

struct stateDescription currentState = {.brakesDeployed = true, .steeringPosition = 0};
struct stateDescription desiredState = {.brakesDeployed = true, .steeringPosition = NEUTRAL_ANGLE};

#if !SERIAL_MODE

//Variables to describe the state of control...
// autonMode | radioConnected | Result
//     0     |        0       |   BRAKE
//     0     |        1       |   RADIO-CONTROLLED
//     1     |        0       |   BRAKE
//     1     |        1       |   AUTONOMOUS-CONTROLLED
bool autonMode = false;
bool radioConnected = false;
long lastConnectTime = 0; //Microseconds since the last time we received a radio transmission.

//Global variables and interrupts for processing radio inputs.
struct radioValue {
  long lastRiseTime;
  long lastFallTime;
  long prevRiseTime;
  int pin;
};
typedef int statusLight;

struct radioValue brakes = {.lastRiseTime = 0, .lastFallTime = 0,
         .prevRiseTime = 0, .pin = RADIO_BRAKE_PIN
};
struct radioValue steering = {.lastRiseTime = 0, .lastFallTime = 0,
         .prevRiseTime = 0, .pin = RADIO_STEERING_PIN
};
struct radioValue auton = {.lastRiseTime = 0, .lastFallTime = 0,
         .prevRiseTime = 0, .pin = RADIO_AUTON_PIN
};
#endif

volatile long encoderTicks = 0;

long lastStatusUpdate = millis();
long UPDATE_INTERVAL = 50; //How many milliseconds before the next time we write to the dynamixel.

long lastAutonFeedback = millis();
long FEEDBACK_INTERVAL = 200; //How many milliseconds between each time we update the auton system on our status.
unsigned long error_message = 0;

void setupWatchDogTimer()
{
  // disable interupts
  cli();
  // enable wdt for every 1 second
  wdt_enable(WDTO_1S);
  // reinable interupts
  sei();
}
// new radio
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

void setup() {
  // new radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  //Setup brake output.
  pinMode(BRAKE_PIN, OUTPUT);
  deployBrakes();

  pinMode(ENCODER_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), incrementEncoderTicks, CHANGE);

#if SERIAL_MODE
  //Setup serial communication with user.
  Serial.begin(9600);
  inputString.reserve(200);

#else
  Serial.begin(RBSM_BAUD);
  first->value = 0;
  first->next = NULL;
#endif

  //Set up communication with Dynamixel Servo
  Dynamixel.begin(DYNAMIXEL_BAUD);
  Dynamixel.setDirectionPin(TRISTATE_TOGGLE_PIN);
  Dynamixel.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE,CCW_LIMIT_ANGLE);
  Dynamixel.setID(SERVO_ID, SERVO_RECEIVE_ID); //Set up the Dynamixel's ID for receiving messages.
  Dynamixel.servo(SERVO_ID, desiredState.steeringPosition, DEFAULT_SPEED); //Steer to a neutral position.

  pinMode(TRISTATE_TOGGLE_PIN, OUTPUT);

#if !SERIAL_MODE
  //Set up radio input.
  //pinMode(RADIO_BRAKE_PIN, INPUT_PULLUP);
  //pinMode(RADIO_STEERING_PIN, INPUT_PULLUP);
  //pinMode(RADIO_AUTON_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(RADIO_BRAKE_PIN), trackBrakeRiseFall, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(RADIO_STEERING_PIN), trackSteeringRiseFall, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(RADIO_AUTON_PIN), trackAutonRiseFall, CHANGE);
#endif

  setupStatusLight(BATTERY_LOW_STATUS_LIGHT);
  setupStatusLight(RC_LOST_SIGNAL);
  setupStatusLight(AUTON_LOST_SIGNAL);
  setupWatchDogTimer();
}

void loop() {
  //Every cycle...
  //... we handle user input, if necessary.
  //SERIAL MODE INPUT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if SERIAL_MODE
  if (stringComplete) {
    Serial.println("RECEIVED:");
    Serial.println(inputString);
    if (inputString.equalsIgnoreCase("brake\n")) desiredState.brakesDeployed = true;
    if (inputString.equalsIgnoreCase("nobrake\n")) desiredState.brakesDeployed = false;

    if (inputString.equalsIgnoreCase("verbose\n")) verboseMode = true;
    if (inputString.equalsIgnoreCase("noverbose\n")) verboseMode = false;

    if (inputString.equalsIgnoreCase("torque\n")) Serial.println(Dynamixel.readLoad(SERVO_RECEIVE_ID), HEX);

    if (inputString.equalsIgnoreCase("ticks\n")) Serial.println(encoderTicks);
    if (inputString.substring(0, 5).equalsIgnoreCase("move "))
    {
      desiredState.steeringPosition = inputString.substring(5).toInt();
      Serial.print("Moving to ");
      Serial.println(desiredState.steeringPosition);
      //Dynamixel.servo(SERVO_ID, desiredState.steeringPosition, DEFAULT_SPEED);
    }
    if (inputString.substring(0, 12).equalsIgnoreCase("torquelimit "))
    {
      int limitTorque = inputString.substring(12).toInt();
      Dynamixel.setMaxTorque(SERVO_RECEIVE_ID, limitTorque);
      Serial.print("Torque Limited to ");
      Serial.println(limitTorque);
    }

    if (inputString.equalsIgnoreCase("state\n"))
    {
      Serial.print("Current Brakes:");
      Serial.println(currentState.brakesDeployed);
      Serial.print("Desired Brakes:");
      Serial.println(desiredState.brakesDeployed);
      Serial.print("Current Steering Position:");
      Serial.println(currentState.steeringPosition);
      Serial.print("Desired Steering Position:");
      Serial.println(desiredState.steeringPosition);
    }

    // clear the string:
    inputString = "";
    stringComplete = false;
  }


#endif


#if !SERIAL_MODE
  //RADIO MODE INPUT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //From the radio rise and fall times, calculate what the current state should be.
#ifndef FORCE_AUTONOMOUS
  // put this into radio functions
  int text[32];
  if (radio.available()) {
    radio.read(&text, sizeof(text));
    brakes.lastFallTime = micros();
    steering.lastFallTime = micros();
    radioConnected = true;
    if(text[0] & (1<<11))
    {
      autonMode = true;
      if(text[0] & (1<<10)) {
        desiredState.brakesDeployed = true;
      }
      else
      {
        desiredState.brakesDeployed = false;
      }
    }
    else
    {
      autonMode = false;
      if(text[0] & (1<<10)) {
        desiredState.brakesDeployed = true;
      }
      else
      {
        desiredState.brakesDeployed = false;
      }
      // 218 to 950 to 975 to 1425
      desiredState.steeringPosition = map(text[0]&0x3FF,180,880,975,1425);
    }
  }
  calculateRadioDesiredState(); 
#endif
  if (verboseMode)
  {
    Serial.print("Steering Pulse Width: ");
    Serial.println(getSteeringPulseWidth());
  }

  //AUTON MODE INPUT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#ifdef FORCE_AUTONOMOUS
  autonMode = true;
#endif
  calculateAutonDesiredState();

  if (millis() - lastAutonFeedback > FEEDBACK_INTERVAL)
  {
    sendAutonFeedback();
    lastAutonFeedback = millis();
  }
#endif

  //SEND COMMANDS TO HARDWARE
  
  //We only update the physical hardware every so often, defined by UPDATE_INTERVAL.
  if (millis() - lastStatusUpdate > UPDATE_INTERVAL)
  {
    //First, we want to read from the dynamixel what its current position is.
    currentState.steeringPosition = Dynamixel.readPosition(SERVO_RECEIVE_ID);
    delay(1);
    if (verboseMode)
    {
      Serial.print("Current steering angle: ");
      Serial.println(currentState.steeringPosition);
    }

    //No matter what, we can update the brake status.
    if (desiredState.brakesDeployed == true && currentState.brakesDeployed == false)
    {
      deployBrakes();
    }
    if (desiredState.brakesDeployed == false && currentState.brakesDeployed == true)
    {
      retractBrakes();
    }
    //If we're more than 25 away from our desired position, we will steer in the right direction as well.
    if (abs(desiredState.steeringPosition - currentState.steeringPosition) > 25)
    {
      //Sometimes, the user will give us crazy stuff, so we bound their desired position by our actual limits.
      int actualTargetAngle = max(CW_LIMIT_ANGLE, desiredState.steeringPosition);
      actualTargetAngle = min(CCW_LIMIT_ANGLE, actualTargetAngle);

      if (verboseMode)
      {
        Serial.print("Steering to... ");
        Serial.println(actualTargetAngle);
      }
      Dynamixel.servo(SERVO_ID, desiredState.steeringPosition, DEFAULT_SPEED);
    }
    //Update the timer.
    lastStatusUpdate = millis();

  }
  // prevent system from reseting
  wdt_reset();
}

void deployBrakes()
{
  if (verboseMode)Serial.println("Deploying Brakes!");
  digitalWrite(BRAKE_PIN, LOW);
  currentState.brakesDeployed = true;
}

void retractBrakes()
{
  if (verboseMode)Serial.println("Retracting Brakes!");
  digitalWrite(BRAKE_PIN, HIGH);
  currentState.brakesDeployed = false;
}

#if SERIAL_MODE
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

#else

//Alternative serialEvent to handle incoming strings from the high-level auton system.
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    addToAutonBuffer((unsigned char)inChar);
  }
}
#endif

void incrementEncoderTicks()
{
  encoderTicks++;
}

void setupStatusLight(statusLight sl)
{
  pinMode(sl, OUTPUT);
  digitalWrite(sl,LOW);
}
void turnOnStatusLight(statusLight sl)
{
  digitalWrite(sl,HIGH);
}
void turnOffStatusLight(statusLight sl)
{
  digitalWrite(sl,LOW);
}

