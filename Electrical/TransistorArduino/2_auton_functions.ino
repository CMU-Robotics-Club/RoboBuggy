#if !SERIAL_MODE

void addToAutonBuffer(unsigned char a)
{
  last->value = a;
  last->next = malloc(sizeof(struct autonBufferElem));
  last->next->value = 0;
  last->next->next = NULL;
  last = last->next;
}

void clearBufferTillMessage()
{
  //first = messageStart->next;
  //free(messageStart);
  //messageStart = first;
  //messageEnd = messageEnd->next;
  //if (messageEnd == NULL) return;
}

void calculateAutonDesiredState()
{
  if (autonMode)
  {
    //If we have new messages from high-level
    if (first == NULL) return;
    struct autonBufferElem* messageEnd = first;
    for (uint8_t i = 0; i < 5; i++)
    {
      messageEnd = messageEnd->next;
      if (messageEnd == NULL) return;
    }

    struct autonBufferElem* messageStart = first;
    struct autonBufferElem* temp = NULL;

    while (messageEnd != NULL)
    {
      if (messageEnd->value == '\n' && (messageStart->value == RBSM_MID_MEGA_STEER_COMMAND || messageStart->value == RBSM_MID_MEGA_AUTON_BRAKE_COMMAND))
      {
        switch (messageStart->value)
        {
          case RBSM_MID_MEGA_STEER_COMMAND:
            desiredState.steeringPosition = (int)(((unsigned long)(messageStart->next->value) << 24)
                                                  | ((unsigned long)(messageStart->next->next->value) << 16)
                                                  | ((unsigned long)(messageStart->next->next->next->value) << 8)
                                                  | ((unsigned long)(messageStart->next->next->next->next->value)));
            break;

          case RBSM_MID_MEGA_AUTON_BRAKE_COMMAND:
            desiredState.brakesDeployed = (boolean)(messageStart->next->next->next->next->value);
            break;
	        default:
            error_message |= 1<<RBSM_EID_RBSM_INVALID_MID;
        }
        for (uint8_t i = 0; i < 5; i++)
        {
          first = messageStart->next;
          free(messageStart);
          messageStart = first;
          messageEnd = messageEnd->next;
          if (messageEnd == NULL) return;
        }

      }
      while(messageEnd->value != '\n' && messageEnd != NULL &&(messageStart->value != RBSM_MID_MEGA_STEER_COMMAND || messageStart->value != RBSM_MID_MEGA_AUTON_BRAKE_COMMAND))
      {
        first = messageStart->next;
        free(messageStart);
        messageStart = first;
        messageEnd = messageEnd->next;
      }
    }
  }
  /*else
  {
    while(first!=NULL)
    {
      struct autonBufferElem* messageStart = first;
      first = first->next;
      free(messageStart);
    }
  }*/
}



void sendMessage(unsigned char header, unsigned long data)
{
  unsigned char message[6] = {header,
                              ((data >> 24) & 0xFF),
                              ((data >> 16) & 0xFF),
                              ((data >> 8) & 0xFF),
                              (data & 0xFF),
                              '\n'
                             };

  for (uint8_t i = 0; i < 6; i++)
  {
    Serial.write(message[i]);
  }
}

void sendAutonFeedback()
{
  sendMessage(RBSM_MID_MEGA_STEER_ANGLE, (unsigned long)desiredState.steeringPosition);
  sendMessage(RBSM_MID_MEGA_BRAKE_STATE, (unsigned long)currentState.brakesDeployed);
  sendMessage(DEVICE_ID, (unsigned long)0); //The Mega's device ID is zero.
  sendMessage(RBSM_MID_MEGA_TELEOP_BRAKE_COMMAND, (unsigned long)desiredState.brakesDeployed);
  sendMessage(RBSM_MID_MEGA_AUTON_BRAKE_COMMAND, (unsigned long)desiredState.brakesDeployed);
  sendMessage(RBSM_MID_MEGA_AUTON_STATE, (unsigned long)autonMode);
  sendMessage(RBSM_MID_MEGA_BATTERY_LEVEL, (unsigned long)42); // :D
  sendMessage(RBSM_MID_MEGA_STEER_FEEDBACK, (unsigned long)currentState.steeringPosition);
  sendMessage(RBSM_MID_ENC_TICKS_RESET, (unsigned long)encoderTicks);
  sendMessage(RBSM_MID_MEGA_TIMESTAMP, millis());
  sendMessage(RBSM_MID_COMP_HASH, (unsigned long)42);
  sendMessage(RBSM_MID_ERROR, (unsigned long)error_message);
  error_message = 0;
}

#endif
