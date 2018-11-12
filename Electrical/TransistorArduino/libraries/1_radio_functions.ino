#ifndef SERIAL_MODE
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
  if(currentValue)
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
  if(val->lastRiseTime > val->lastFallTime)
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
  if(micros() - brakes.lastFallTime > 100000 || micros() - steering.lastFallTime > 100000) 
  {
    radioConnected = false;
    autonMode = false;
    desiredState.brakesDeployed = true; //We always deploy the brakes when we lose connection.
    desiredState.steeringPosition = NEUTRAL_ANGLE;
    return;
  }

  //If not, are we in autonomous mode?
  if(getAutonPulseWidth() > 1500)
  {
    autonMode = true;
    return;
  }
  else
  {
    autonMode = false;
  }
  
  //If not, then should the brakes be dropped?
  if(getBrakePulseWidth() < 1500)
  {
    desiredState.brakesDeployed = true;
  }
  else
  {
    desiredState.brakesDeployed = false;
  }

  //We need to map an angle from 1000 to 2000 onto a range from 1950 to 2550
  desiredState.steeringPosition = (((getSteeringPulseWidth() - 1050)*2)/3) + 1950; //NEED TO DETERMINE THESE VALUES EXPERIMENTALLY.
  if(verboseMode)
  {
    Serial.print("Calculated Steering Angle: ");
    Serial.println(desiredState.steeringPosition);
  }
  
}


#endif
