/*

Version 2.2

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */

#include "Dynamixel_Serial.h"

//##############################################################################
//############################ Public Methods ##################################
//##############################################################################

void DynamixelClass::begin(long baud){
    uart1_init(UART_BAUD_SELECT(baud, F_CPU));
    uart1_fdevopen(_steering_uart);

    //TODO: Setup pin here

}



unsigned int DynamixelClass::reset(unsigned char ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = RESET_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_RESET;

    clearRXbuffer();

    transmitInstructionPacket();

    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }

}

unsigned int DynamixelClass::ping(unsigned char ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = PING_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_PING;

    clearRXbuffer();

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
        return (Status_Packet_Array[0]);            // Return SERVO ID
    }else{
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
    }

}


unsigned int DynamixelClass::setMode(unsigned char ID, unsigned int Dynamixel_CW_Limit,unsigned int Dynamixel_CCW_Limit){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_MODE_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_CW_ANGLE_LIMIT_L;

    //set SERVO mode
    Instruction_Packet_Array[4] = (uint8_t)(Dynamixel_CW_Limit);
    Instruction_Packet_Array[5] = (uint8_t)((Dynamixel_CW_Limit & 0x0F00) >> 8);
    Instruction_Packet_Array[6] = (uint8_t)(Dynamixel_CCW_Limit);
    Instruction_Packet_Array[7] = (uint8_t)((Dynamixel_CCW_Limit & 0x0F00) >> 8);

    clearRXbuffer();

    transmitInstructionPacket();

    if (Status_Return_Value == ALL)
    {
        readStatusPacket();
        if (Status_Packet_Array[2] != 0)
        {
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
 }


unsigned int DynamixelClass::servo(unsigned char ID,unsigned int Position,unsigned int Speed){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[4] = (uint8_t)(Position);
    Instruction_Packet_Array[5] = (uint8_t)((Position & 0x0F00) >> 8);
    Instruction_Packet_Array[6] = (uint8_t)(Speed);
    Instruction_Packet_Array[7] = (uint8_t)((Speed & 0x0F00) >> 8);

    clearRXbuffer();

    transmitInstructionPacket();


    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }

}


unsigned int DynamixelClass::readPosition(unsigned char ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_POS_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_PRESENT_POSITION_L;
    Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;

    clearRXbuffer();

    transmitInstructionPacket();
    readStatusPacket();

    if (Status_Packet_Array[2] == 0) // If there is no status packet error return value 
    {               
        return Status_Packet_Array[4] << 8 | Status_Packet_Array[3];    // Return present position value
    }
    else
    {
        return (Status_Packet_Array[2] | 0xF000);                           // If there is a error Returns error value
    }
}



//##############################################################################
//########################## Private Methods ###################################
//##############################################################################

void DynamixelClass::transmitInstructionPacket(void){                                   // Transmit instruction packet to Dynamixel

    if (Direction_Pin > -1)
    {
        digitalWrite(Direction_Pin,HIGH);                                               // Set TX Buffer pin to HIGH
    }

    _steering_uart->write(HEADER);                                                             // 1 Write Header (0xFF) data 1 to serial
    _steering_uart->write(HEADER);                                                             // 2 Write Header (0xFF) data 2 to serial
    _steering_uart->write(Instruction_Packet_Array[0]);                                        // 3 Write Dynamixal ID to serial
    _steering_uart->write(Instruction_Packet_Array[1]);                                        // 4 Write packet length to serial
    _steering_uart->write(Instruction_Packet_Array[2]);                                        // 5 Write instruction type to serial

    unsigned int checksum_packet = Instruction_Packet_Array[0] + Instruction_Packet_Array[1] + Instruction_Packet_Array[2];

    for (unsigned char i = 3; i <= Instruction_Packet_Array[1]; i++){
        _steering_uart->write(Instruction_Packet_Array[i]);                                    // Write Instuction & Parameters (if there are any) to serial
        checksum_packet += Instruction_Packet_Array[i];
    }

    noInterrupts();

    _steering_uart->write(~checksum_packet & 0xFF);                                            // Write low bit of checksum to serial

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__) // Leonardo and Mega use Serial1
    if ((UCSR1A & B01100000) != B01100000){                                             // Wait for TX data to be sent
        _steering_uart->flush();
    }

#elif defined(__SAM3X8E__)

    //if(USART_GetFlagStatus(USART1, USART_FLAG_TC) != RESET)
        _steering_uart->flush();
    //}

#else
    if ((UCSR0A & B01100000) != B01100000){                                             // Wait for TX data to be sent
        _steering_uart->flush();
    }

#endif

    if (Direction_Pin > -1){
        digitalWrite(Direction_Pin,LOW);                                                //Set TX Buffer pin to LOW after data has been sent
    }

    interrupts();

}


unsigned int DynamixelClass::readStatusPacket(void){

    unsigned char Counter = 0x00;
    unsigned char First_Header = 0x00;

    Status_Packet_Array[0] = 0x00;
    Status_Packet_Array[1] = 0x00;
    Status_Packet_Array[2] = 0x00;
    Status_Packet_Array[3] = 0x00;

    Time_Counter = STATUS_PACKET_TIMEOUT + millis();                 // Setup time out error

    while(STATUS_FRAME_BUFFER >= _steering_uart->available()){       // Wait for " header + header + frame length + error " RX data

        if (millis() >= Time_Counter)
        {
            return Status_Packet_Array[2] = B10000000;                                      // Return with Error if Serial data not received with in time limit
        }
    }

    if (_steering_uart->peek() == 0xFF && First_Header != 0xFF)
    {
        First_Header = _steering_uart->getc();                                                 // Clear 1st header from RX buffer
    }
    else if (_steering_uart->peek() == -1)
    {
        return Status_Packet_Array[2] = B10000000;                                      // Return with Error if two headers are not found
    }

    if(_steering_uart->peek() == 0xFF && First_Header == 0xFF){
        _steering_uart->read();                                                                // Clear 2nd header from RX buffer
        Status_Packet_Array[0] = _steering_uart->read();                                   // ID sent from Dynamixel
        Status_Packet_Array[1] = _steering_uart->read();                                       // Frame Length of status packet
        Status_Packet_Array[2] = _steering_uart->read();                                       // Error byte

        Time_Counter = STATUS_PACKET_TIMEOUT + millis();
        while(Status_Packet_Array[1] - 2 >= _steering_uart->available()){              // Wait for wait for "Para1 + ... Para X" received data

            if (millis() >= Time_Counter){
                return Status_Packet_Array[2] = B10000000;                          // Return with Error if Serial data not received with in time limit
            }
        }
        do{
            Status_Packet_Array[3 + Counter] = _steering_uart->read();
            Counter++;
        }while(Status_Packet_Array[1] > Counter);                           // Read Parameter(s) into array

        Status_Packet_Array[Counter + 4] = _steering_uart->read();                         // Read Check sum

    }else{
        return Status_Packet_Array[2] = B10000000;                                      // Return with Error if two headers are not found
    }
}

void DynamixelClass::clearRXbuffer(void){

    while (_steering_uart->read() != -1);  // Clear RX buffer;

}

DynamixelClass Dynamixel;
