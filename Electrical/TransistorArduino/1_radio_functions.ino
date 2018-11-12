#if !SERIAL_MODE
#define radio_loss_time 500000
void trackSteeringRiseFall()
{
  trackRiseFall(&steering);
}

void trackBrakeRiseFall()
{
  trackRiseFall(&brakes);
}

void trackAutonRiseFall()
{
  trackRiseFall(&auton);
}

void trackRiseFall(struct radioValue* val)
{
  long currentTime = micros();
  int currentValue = digitalRead(val->pin);
  if (currentValue)
  {
    //If it's a rising value...
    val->prevRiseTime = val->lastRiseTime;
    val->lastRiseTime = currentTime;
  }
  else
  {
    //If it's a falling value...
    val->lastFallTime = currentTime;
    radioConnected = true;
  }

}

long getBrakePulseWidth()
{
  return getPulseWidth(&brakes);
}

long getSteeringPulseWidth()
{
  return getPulseWidth(&steering);
}

long getAutonPulseWidth()
{
  return getPulseWidth(&auton);
}

long getPulseWidth(struct radioValue* val)
{
  if (val->lastRiseTime > val->lastFallTime)
  {
    return val->lastFallTime - val->prevRiseTime;
  }
  else
  {
    return val->lastFallTime - val->lastRiseTime;
  }
}

void calculateRadioDesiredState()
{
  //Is the radio disconnected?
  if (micros() - brakes.lastFallTime > radio_loss_time || micros() - steering.lastFallTime > radio_loss_time)
  {
    radioConnected = false;
    autonMode = false;
    error_message |= 1<<RBSM_EID_RC_LOST_SIGNAL;
    turnOnStatusLight(RC_LOST_SIGNAL);
    desiredState.brakesDeployed = true; //We always deploy the brakes when we lose connection.
    desiredState.steeringPosition = NEUTRAL_ANGLE;
    return;
  }

  turnOffStatusLight(RC_LOST_SIGNAL);
  error_message &= ~(1<<RBSM_EID_RC_LOST_SIGNAL);
 
  //If not, are we in autonomous mode?
  /*if (getAutonPulseWidth() > 1500)
  {
    autonMode = true;
    return;
  }
  else
  {
    autonMode = false;
  }*/

  //If not, then should the brakes be dropped?
  /*if (getBrakePulseWidth() < 1500)
  {
    desiredState.brakesDeployed = true;
  }
  else
  {
    desiredState.brakesDeployed = false;
  }*/
  //We need to map an angle from 1000 to 2000 onto a range from 975 to 1425 
  //desiredState.steeringPosition = ((getSteeringPulseWidth() - 1000)/2) + 975;//NEED TO DETERMINE THESE VALUES EXPERIMENTALLY.
  
  if(verboseMode)
  {
    Serial.print("Calculated Steering Angle: ");
    Serial.println(desiredState.steeringPosition);
  }

}


#endif
