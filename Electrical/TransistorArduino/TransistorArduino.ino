#include "Dynamixel_Serial.h"

#include <avr/wdt.h>


//Constants to do with the Dynamixel.
#define SERIAL_MODE 0
#define FORCE_AUTONOMOUS false //This should only be defined for debug purposes.
#define SEND_AUTON_FEEDBACK true
#define NO_DYNAMIXEL false
#define HIGH_LEVEL_INTER false
#define DYNAMIXEL_BAUD 1000000
#define SERVO_ID 0xFE
#define SERVO_RECEIVE_ID 0x01 //0x04
#define CW_LIMIT_ANGLE 975
#define CCW_LIMIT_ANGLE 1425

#define RADIO_MESSAGE_MAX 24 // extra 2 for new line

#define NEUTRAL_ANGLE 1200
#define DEFAULT_SPEED 0x100


//Define the pin numbering scheme.
#define RADIO_STEERING_PIN 2
#define RADIO_BRAKE_PIN 21
#define RADIO_AUTON_PIN 20
#define BRAKE_PIN 31
#define TRISTATE_TOGGLE_PIN 3
#define ENCODER_PIN 21
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

#define steer_pin 2

// new radio
uint16_t channels[20];
uint16_t zeros[4] = {0,0,0,0};//{991,991,991,991};
boolean en = false;
boolean failsafe = true;
uint32_t timeOfLastRx = 0;

int time_taken = 0;
int start_time = millis();
int end_time = 0;
int messages_received = 0;

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
long UPDATE_INTERVAL = 100; //How many milliseconds before the next time we write to the dynamixel.

long lastAutonFeedback = millis();
long FEEDBACK_INTERVAL = 200; //How many milliseconds between each time we update the auton system on our status.
unsigned long error_message = 0;

int autonFeedbackPart = 0;

void setupWatchDogTimer()
{
  // disable interupts
  cli();
  // enable wdt for every 1 second
  wdt_enable(WDTO_1S);
  // reinable interupts
  sei();
}


void setup() {
  //Setup brake output.
  pinMode(BRAKE_PIN, OUTPUT);
  deployBrakes();
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  digitalWrite(10,0);
  digitalWrite(11,0);
  pinMode(ENCODER_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), incrementEncoderTicks, RISING);
#if HIGH_LEVEL_INTER
  attachInterrupt(digitalPinToInterrupt(steer_pin), changeSteering, CHANGE);
#endif
  

#if SERIAL_MODE
  //Setup serial communication with user.
  Serial.begin(9600);
  inputString.reserve(200);
#else
//#if SEND_AUTON_FEEDBACK
  Serial.begin(RBSM_BAUD);
//#endif
  first->value = 0;
  first->next = NULL;
   Serial2.begin(100000, SERIAL_8E2);
#endif

  //Set up communication with Dynamixel Servo
#if !NO_DYNAMIXEL
  Dynamixel.begin(DYNAMIXEL_BAUD);
  Dynamixel.setDirectionPin(TRISTATE_TOGGLE_PIN);
  Dynamixel.setHoldingTorque(SERVO_ID, true);
  Dynamixel.setMode(SERVO_ID, SERVO, CW_LIMIT_ANGLE,CCW_LIMIT_ANGLE);
  Dynamixel.setID(SERVO_ID, SERVO_RECEIVE_ID); //Set up the Dynamixel's ID for receiving messages.
  Dynamixel.servo(SERVO_ID, desiredState.steeringPosition, DEFAULT_SPEED); //Steer to a neutral position.
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
#if !NO_DYNAMIXEL
    if (inputString.equalsIgnoreCase("torque\n")) Serial.println(Dynamixel.readLoad(SERVO_RECEIVE_ID), HEX);
#endif
    if (inputString.equalsIgnoreCase("ticks\n")) Serial.println(encoderTicks);
    if (inputString.substring(0, 5).equalsIgnoreCase("move "))
    {
      desiredState.steeringPosition = inputString.substring(5).toInt();
      Serial.print("Moving to ");
      Serial.println(desiredState.steeringPosition);
    }
    if (inputString.substring(0, 12).equalsIgnoreCase("torquelimit "))
    {
      int limitTorque = inputString.substring(12).toInt();
#if !NO_DYNAMIXEL
      Dynamixel.setMaxTorque(SERVO_RECEIVE_ID, limitTorque);
#endif
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
  // get timing for diagnostic
  end_time = micros();
  time_taken = end_time - start_time;
  start_time = end_time;
  //RADIO MODE INPUT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //From the radio rise and fall times, calculate what the current state should be.
#if !FORCE_AUTONOMOUS
  // put this into radio functions
  static uint32_t serial_pd = 0;
  if(millis()-serial_pd > 100){
    serial_pd=millis();
    digitalWrite(13,LOW);
  }
  if(millis()-timeOfLastRx > 100){
    failsafe = true;
    en=false;
  }
  if(failsafe){
    digitalWrite(13,(millis()%200)>100);
  }
  
  boolean doExit = false;
  //SBUS decoder
  uint32_t start = millis();
  uint32_t loopStart = millis();
  messages_received = 0;
  while(Serial2.available() && !doExit && (millis()-loopStart < 40)) {
    //digitalWrite(13,HIGH);
    uint32_t maxTime = 20;
    if(millis()-start > maxTime){
      doExit = true;
      break;
    }
    uint8_t srd = Serial2.read();
    if(srd == 0x0F){
      start = millis();
      while(!Serial2.available() && !doExit){
        if(millis()-start > maxTime){
          doExit=true;
          break;
        }
      }
      //read in data
      uint8_t bytes[25];
      for(int i=0;i<22 && !doExit;i++){
        while(!Serial2.available() && !doExit){
          if(millis()-start > maxTime){
            doExit=true;
            //Serial.println("not enough data");
            break;
          }
        }
        bytes[i] = Serial2.read();
        messages_received++;
      }
      while(!Serial2.available() && !doExit){
        if(millis()-start > maxTime){
          doExit=true;
        }
      }
      byte fs = Serial2.read();
      messages_received++;
      while(!Serial2.available() && !doExit){
        if(millis()-start > maxTime){
          doExit=true;
        }
      }
      byte footer = Serial2.read();
      messages_received++;
      //seems like the parser is catching Fs in the datastream as start bytes, 
      //but channel 4 shows up in two places, so compare those against each other
      uint16_t chan4 = (bytes[5]>>4) | ((uint16_t)(bytes[6]&0x7F))<<4;
      if(chan4 != 172 && chan4!=1811){
        doExit=true;
        digitalWrite(10,1);
      }else{
        digitalWrite(10,0);
      }
      if(footer == 0 && !doExit){
        if(fs & 0x08){
          failsafe = true;
          digitalWrite(11,1);
          en=false;
        }else{
          failsafe=false;
          digitalWrite(11,0);
        }
        timeOfLastRx = millis();
        channels[0] = bytes[0] | ((uint16_t)(bytes[1] & 0x7))<<8;
        channels[1] = (bytes[1]>>3) | ((uint16_t)(bytes[2]&0x3F))<<5;
        channels[2] = (bytes[2]>>6) | ((uint16_t)bytes[3])<<2 | ((uint16_t)bytes[4]&1)<<10;
        channels[3] = (bytes[4]>>1) | ((uint16_t)(bytes[5]&0xF))<<7;
        channels[4] = (bytes[5]>>4) | ((uint16_t)(bytes[6]&0x7F))<<4;
        channels[5] = (bytes[6]>>7) | ((uint16_t)(bytes[7]))<<1 | ((uint16_t)bytes[8]&0x3)<<9;
        doExit=true;
        if(channels[1]!=0) // radio message received
        {
          
          brakes.lastFallTime = micros();
          steering.lastFallTime = micros();
          if(channels[4]<1800)
          {
            desiredState.brakesDeployed = true;
          }
          else
          {
            desiredState.brakesDeployed = false;
          }
          // auton is at either 1299 or 275, it switches sometimes
          if(channels[5]<200)
          {
            autonMode = true;
          }
          else
          {
            autonMode = false;
            int remap_steering = 1811+172-channels[1];
            desiredState.steeringPosition = map(remap_steering,172,1811,975,1425);
          }
        }
        
      }
    }
  }
  if(doExit){
    for(int i=0;i<60;i++){
      if(!Serial2.available()){
        i=100;
        break;
      }
      Serial2.read();
      messages_received++;
    }
  }
  calculateRadioDesiredState(); 
#endif

  //AUTON MODE INPUT~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if FORCE_AUTONOMOUS
  autonMode = true;
#endif
  if (millis() - lastAutonFeedback > FEEDBACK_INTERVAL)
  {
    if(autonFeedbackPart == 11)
    {
#if SEND_AUTON_FEEDBACK
      sendAutonFeedback(autonFeedbackPart);
#endif
      autonFeedbackPart++;
    }
    else
    {
      autonFeedbackPart = 0;
    }
    lastAutonFeedback = millis();
  }
  if(autonFeedbackPart < 11)
  {
#if SEND_AUTON_FEEDBACK
      sendAutonFeedback(autonFeedbackPart);
#endif
    autonFeedbackPart++;
  }
  else if(autonFeedbackPart > 6 && autonFeedbackPart < 12)
  {
#if SEND_AUTON_FEEDBACK
      sendAutonFeedback(autonFeedbackPart);
#endif
    autonFeedbackPart++;
    if(autonFeedbackPart <= 12)
    {
      autonFeedbackPart = 0;
    }
  }
#endif

  //SEND COMMANDS TO HARDWARE
  
  //We only update the physical hardware every so often, defined by UPDATE_INTERVAL.
  if (millis() - lastStatusUpdate > UPDATE_INTERVAL)
  {
    //First, we want to read from the dynamixel what its current position is.
#if !NO_DYNAMIXEL
    // since we have no indication of actual steering position
    //currentState.steeringPosition = desiredState.steeringPosition;//Dynamixel.readPosition(SERVO_RECEIVE_ID);
#endif
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
    calculateAutonDesiredState();
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
#if !NO_DYNAMIXEL
      Dynamixel.servo(SERVO_ID, actualTargetAngle, DEFAULT_SPEED);
#endif
       currentState.steeringPosition = desiredState.steeringPosition;
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
#if SEND_AUTON_FEEDBACK
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    addToAutonBuffer((unsigned char)inChar);
  }
}
#endif
#endif

#if HIGH_LEVEL_INTER
void changeSteering()
{
  int this_time = micros();
  int this_state = digitalRead(steer_pin);
  if(this_state == HIGH)
  {
    steering.lastRiseTime = this_time;
  }
  else
  {
    steering.lastFallTime = this_time;
    steering.prevRiseTime = steering.lastFallTime - steering.lastRiseTime;
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

