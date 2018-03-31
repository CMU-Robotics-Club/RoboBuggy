/*

NANDistor

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
#include "system_clock.h"

//##############################################################################
//############################ Public Methods ##################################
//##############################################################################

void DynamixelClass::begin(long baud){
    uart1_init(UART_BAUD_SELECT(baud, F_CPU));
    uart1_fdevopen(_steering_uart);

    PORTE &= ~_BV(PE5);
    DDRE |= _BV(PE5);
}

int DynamixelClass::Init(UARTFILE *in_file, FILE *out_file) {
    // setup the hardware serial
    _steering_uart = in_file;
    out_file_ = out_file;

    // init the input buffer
    InitReadBuffer();

    // success
    return 1;
}

int DynamixelClass::send(char id, char message) {
    /*char buffer_pos;
    buffer_pos = InitMessageBuffer();
    buffer_pos = AppendMessageToBuffer(id, message, buffer_pos);

    for(int i = 0; i < buffer_pos; i++) {
        fputc(buffer_out_[i], out_file_);
    }

    // success*/
    //fprintf(_steering_uart, "test123456789\r\n");
    // fputc('t', _steering_uart);
    // fputc('e', _steering_uart);
    // fputc('s', _steering_uart);
    // fputc('t', _steering_uart);
    char messages[8];
    messages[0] = 0x4;
    messages[1] = 0x07;
    messages[2] = 0x04;
    messages[3] = 0x1E;
    messages[4] = 0xFD;
    messages[5] = 0x07;
    messages[6] = 0xFF;
    messages[7] = 0x03;
    fprintf(_steering_uart, messages);
    return 1;
}

int DynamixelClass::read(servo_message_t* read_message) {
    while(_steering_uart->available() > 0) {
        // check if there is new data
        char new_serial_byte = fgetc(_steering_uart);

        /*// first, we need to try to lock on to the stream
        if(buffer_in_stream_lock_ == false) {
            printf("%s: searching for lock...\n", __PRETTY_FUNCTION__);
            if((char)new_serial_byte == FOOTER) {
                printf("%s: got lock!\n", __PRETTY_FUNCTION__);
                buffer_in_stream_lock_ = true;
            }*/
        // after we lock we need to read in to the buffer until full
        //} else {
            // this->Send(RBSM_MID_ERROR, buffer_in_pos_);
            buffer_in_[buffer_in_pos_] = (char)new_serial_byte; 
            buffer_in_pos_++;
            // handle the end of a packet
            if(buffer_in_pos_ == SERVO_PACKET_LENGTH) {
                // reset buffer for next packet
                buffer_in_pos_ = 0;
                // parse this complete packet
                //if(buffer_in_[5] == FOOTER) {
                    read_message->message_id = buffer_in_[0];
                    char *data_bytes = (char *) &(read_message->data);
                    data_bytes[0] = buffer_in_[4];
                    data_bytes[1] = buffer_in_[3];
                    data_bytes[2] = buffer_in_[2];
                    data_bytes[3] = buffer_in_[1];
                    return 1;
                // skip packet as an error
                //} else {
                //    buffer_in_stream_lock_ = false;
                //    return RBSM_ERROR_INVALID_MESSAGE;
                //}
            } 
        //}
    }
    // we didn't have enough data to read a whole packet
    return SERVO_ERROR_INSUFFICIENT_DATA;

    // maybe check if the footer matches buffer_in_, else throw the message cause it is sad.
}

char DynamixelClass::AppendMessageToBuffer(char id,
                                                char message,
                                                char out_start_pos) {
    char buffer_pos = out_start_pos;
    char message_ll = message;// & RBSM_ONE_BYTE_MASK;
    /*char message_lh = (message >> RBSM_ONE_BYTE_SIZE) & RBSM_ONE_BYTE_MASK;
    char message_hl = (message >> RBSM_TWO_BYTE_SIZE) & RBSM_ONE_BYTE_MASK;
    char message_hh = (message >> RBSM_THREE_BYTE_SIZE) & RBSM_ONE_BYTE_MASK;*/

    // write message id (and id since single message)
/*    buffer_out_[buffer_pos++] = id;

    buffer_out_[buffer_pos++] = message_hh;
    buffer_out_[buffer_pos++] = message_hl;
    buffer_out_[buffer_pos++] = message_lh; */
    buffer_out_[buffer_pos++] = message_ll;

    //buffer_out_[buffer_pos++] = FOOTER;

    // write null terminator just in case. note no increment
    //buffer_out_[buffer_pos] = RBSM_NULL_TERM;


    return buffer_pos;
}

char DynamixelClass::InitMessageBuffer() {
    char buffer_pos = 0;

    // write null terminator just in case. note no increment
    //buffer_out_[buffer_pos] = RBSM_NULL_TERM;

    return buffer_pos;
}


int DynamixelClass::InitReadBuffer() {
    buffer_in_pos_ = 0;
    //buffer_in_stream_lock_ = false;
    return 0;
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


// unsigned int DynamixelClass::setMode(unsigned char ID, unsigned int Dynamixel_CW_Limit,unsigned int Dynamixel_CCW_Limit){

//     Instruction_Packet_Array[0] = ID;
//     Instruction_Packet_Array[1] = SET_MODE_LENGTH;
//     Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
//     Instruction_Packet_Array[3] = EEPROM_CW_ANGLE_LIMIT_L;

//     //set SERVO mode
//     Instruction_Packet_Array[4] = (char)(Dynamixel_CW_Limit);
//     Instruction_Packet_Array[5] = (char)((Dynamixel_CW_Limit & 0x0F00) >> 8);
//     Instruction_Packet_Array[6] = (char)(Dynamixel_CCW_Limit);
//     Instruction_Packet_Array[7] = (char)((Dynamixel_CCW_Limit & 0x0F00) >> 8);

//     clearRXbuffer();

//     transmitInstructionPacket();

//     if (Status_Return_Value == ALL)
//     {
//         readStatusPacket();
//         if (Status_Packet_Array[2] != 0)
//         {
//             return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
//         }
//     }
//  }


unsigned int DynamixelClass::servo(unsigned char ID,unsigned int Position,unsigned int Speed)
{
    PORTE &= ~ (_BV(PE5));

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[4] = (char)(Position);
    Instruction_Packet_Array[5] = (char)((Position & 0x0F00) >> 8);
    Instruction_Packet_Array[6] = (char)(Speed);
    Instruction_Packet_Array[7] = (char)((Speed & 0x0F00) >> 8);
    Instruction_Packet_Array[8] = 0x00; // null terminator since we are printf-ing

    fprintf(_steering_uart, Instruction_Packet_Array);


    PORTE |= _BV(PE5);


    return 0;
    //TODO implement reading the status packet
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

    D_PIN_PORT |= _BV(D_PINN);

    send(D_PINN,HEADER);                                                             // 1 Write Header (0xFF) data 1 to serial
    send(D_PINN,HEADER);                                                             // 2 Write Header (0xFF) data 2 to serial
    send(D_PINN,Instruction_Packet_Array[0]);                                        // 3 Write Dynamixal ID to serial
    send(D_PINN,Instruction_Packet_Array[1]);                                        // 4 Write packet length to serial
    send(D_PINN,Instruction_Packet_Array[2]);                                        // 5 Write instruction type to serial

    unsigned int checksum_packet = Instruction_Packet_Array[0] + Instruction_Packet_Array[1] + Instruction_Packet_Array[2];

    for (unsigned char i = 3; i <= Instruction_Packet_Array[1]; i++){
        send(D_PINN, Instruction_Packet_Array[i]);                                    // Write Instuction & Parameters (if there are any) to serial
        checksum_packet += Instruction_Packet_Array[i];
    }

    cli();

    send(D_PINN, ~checksum_packet & 0xFF);                                            // Write low bit of checksum to serial

    Time_Counter = STATUS_PACKET_TIMEOUT + millis();

    while(millis() < Time_Counter){}

/*#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__) // Leonardo and Mega use Serial1
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

#endif*/

    D_PIN_PORT &= ~ (_BV(D_PINN));

    sei();

}


unsigned int DynamixelClass::readStatusPacket(void){

    unsigned char Counter = 0x00;
    unsigned char First_Header = 0x00;

    Status_Packet_Array[0] = 0x00;
    Status_Packet_Array[1] = 0x00;
    Status_Packet_Array[2] = 0x00;
    Status_Packet_Array[3] = 0x00;

    /*Time_Counter = STATUS_PACKET_TIMEOUT + millis();                 // Setup time out error

    while(STATUS_FRAME_BUFFER >= _steering_uart->available()){       // Wait for " header + header + frame length + error " RX data

        if (millis() >= Time_Counter)
        {
            return Status_Packet_Array[2] = B10000000;                                      // Return with Error if Serial data not received with in time limit
        }
    }*/
    Time_Counter = STATUS_PACKET_TIMEOUT + millis();

    while(millis() < Time_Counter){}


    /*if (_steering_uart->peek() == 0xFF && First_Header != 0xFF)
    {
        First_Header = _steering_uart->getc();                                                 // Clear 1st header from RX buffer
    }
    else if (_steering_uart->peek() == -1)
    {
        return Status_Packet_Array[2] = B10000000;                                      // Return with Error if two headers are not found
read(servo_message_t) != 0xFF) {
        return Status_Packet_Array[2] = B10000000read(servo_message_t) != 0xFF) {
        return Status_Packet_Array[2] = B10000000;
    }

    /*if(_steering_uart->peek() == 0xFF && First_Header == 0xFF){
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
    }   */

    read((servo_message_t*)(&Status_Packet_Array[0]));                                   // ID sent from Dynamixel
    read((servo_message_t*)&Status_Packet_Array[1]);                                       // Frame Length of status packet
    read((servo_message_t*)&Status_Packet_Array[2]);                                   // Error byte

        do{
            read((servo_message_t*)(&Status_Packet_Array[3 + Counter]));
            Counter++;
        }while(Status_Packet_Array[1] > Counter);                           // Read Parameter(s) into array

        read((servo_message_t*)&Status_Packet_Array[Counter + 4]);                         // Read Check sum
    return 0;
}

void DynamixelClass::clearRXbuffer(void){
    servo_message_t dummy;

    while (read(&dummy) != 0);  // Clear RX buffer;

}

DynamixelClass Dynamixel;
