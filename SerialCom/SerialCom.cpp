#include "SerialCom.h"

SerialCom::SerialCom(PinName tx, PinName rx) : BufferedSerial(tx, rx){
    baud(115200);
    current_read_pos = 0;
    message_available = 0;
}


/*!
  Method to read a buffer of chars
  The input message should end with '\n'
  Message can be compiled during several passes

*/
char* SerialCom::getMessage(){
    if (message_available == 1) return message_buffer;
    while(readable())
    {
        char c = getc();
        if (c == '\n' || current_read_pos >= SC_STRING_LENGTH - 1){
            message_buffer[current_read_pos] = '\0';
            message_available = 1;
            return message_buffer;
        }
        else {
            message_buffer[current_read_pos++] = c;
        }
    }
    return NULL;
}


void SerialCom::releaseMessage(){
    message_available = 0;
    current_read_pos = 0;   
}

void SerialCom::stateCheck(){
    NVIC_SystemReset();
}




