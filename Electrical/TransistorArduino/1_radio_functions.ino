#if !SERIAL_MODE
#define radio_loss_time 100000

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

  if(verboseMode)
  {
    Serial.print("Calculated Steering Angle: ");
    Serial.println(desiredState.steeringPosition);
  }

}


#endif
