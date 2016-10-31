//
// File: IOBoard.ino
// Description: IO Board firmware
//
// History
// Local versions
// 06/05/2016 v0.3  Fix readINPin issue.
// 02/10/2016 v0.4  Fix servo issue.
// 28/10/2016 v0.5  Refactoring for Arduino 1.7.x.
//

// include the library code:
#include "IOBoardFirmware.h"

// from https://forum.arduino.cc/index.php?topic=112261.msg844259#msg844259
// Only files with the .ino (or .pde) extension are scanned for include files.
// The IDE copies files to a temporary directory where it performs the build.
#ifndef __IOBoardFirmware__
#include <Arduino.h>
#include <string.h>
#include <Servo.h>
#include <Wire.h>
#include <rgb_lcd.h>
//#include <DynamixelSerial1.h>
#endif


// define main parameters
#define LOOPPERIOD 50      //ms
bool configMode = false;
bool ackEnabled = true;
bool warnErr    = false;
unsigned long nextLoopTime = 0;
uint8_t loopCounter = 0;
uint8_t minLoopFreeTime = 255;

// define containers for assignations
uint8_t IOAssignation[IONUM] = {0};
uint8_t AINAssignation[AINNUM] = {0};
uint8_t AINThreshold[AINNUM] = {0};
uint16_t minMaxServo[PWMNUM] = {0};

// define Serial stats & registers
unsigned short RxStat = 0;
unsigned short RxErr = 0;
unsigned short TxStat = 0;
uint8_t lastMsgID = 0;
uint8_t* RxBuff = (uint8_t*)calloc(RXBUFFLEN, sizeof(uint8_t)); // max serial data payload len + 1
uint8_t* TxBuff = (uint8_t*)calloc(TXBUFFLEN, sizeof(uint8_t));
uint8_t* errorBuff = (uint8_t*)calloc(ERRORBUFFLEN, sizeof(uint8_t));
unsigned short errBuffLen = 0;
unsigned short errBuffIndex = 0;
unsigned short errCount = 0;

// define LCD parameters
#ifdef LCDUSED
rgb_lcd lcd;
uint8_t displayMode = 0x00;
uint8_t* LCDBuff = (uint8_t*)calloc(16, sizeof(uint8_t));
#endif

// TODO a watchdog...

// ######################## SYSTEM FUNCTIONS ########################

void haltFunc() {
#ifdef LEDSTATUSINDEX
    digitalWrite((unsigned short)IOList[LEDSTATUSINDEX], LOW);
#endif
    while(true){};}

void(* resetFunc) (void) = 0;   //declare reset function at address 0 == goto 0 address.

void setup (void) {
    // put your setup code here, to run once:

    // set pin for LED status
#ifdef LEDSTATUSINDEX
    IOAssignation[LEDSTATUSINDEX] = OUTMODE;
    pinMode((unsigned short)IOList[LEDSTATUSINDEX], OUTPUT);
    digitalWrite((unsigned short)IOList[LEDSTATUSINDEX], HIGH);
#endif
#ifdef UIBUTTONINDEX
    IOAssignation[UIBUTTONINDEX] = INMODE;
    pinMode((unsigned short)IOList[UIBUTTONINDEX], INPUT_PULLUP);
#endif

#ifdef LCDUSED
    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    lcd.setRGB(255, 255, 255);
    // initialize the serial communications:
    lcd.clear();
    lcd.write("Boot on-going...");
    lcd.setCursor(0, 1);
    lcd.write("Wainting serial ");
#endif

    Serial.begin(115200);
    while (!Serial.available() && millis() < 15000) {
        delay(100); // wait for serial port to connect. Needed for native USB port only
    }
    if (Serial.available()) {
        delay(500);
        while(Serial.available()) {Serial.read();}  // flush input serial buffer of previous data
#ifdef LCDUSED
        lcd.setCursor(0, 1);
        lcd.write("Serial ready    ");
#endif
        Serial.println("Ready");
    }
    else {
#ifdef LCDUSED
        lcd.setRGB(255, 0, 0);
        lcd.setCursor(0, 1);
        lcd.write("Serial failure  ");
#endif
        delay(5000);
        haltFunc();
    }
    delay(500);
}

void teardone (void) {
    releaseAllServos();
    processOut(0, false, true);
#ifdef LCDUSED
    lcd.clear();
    lcd.setRGB(255, 0, 0);
    lcd.setCursor(0, 0);
    lcd.write("     System     ");
    lcd.setCursor(0, 1);
    lcd.write("    halted !   ");
#endif
}

// ######################## DATA MANAGEMENT ########################

String formatIntStrLen(const unsigned short val, const uint8_t outlen, uint8_t maxlen = 0) {
    String ret = String(val);
    uint8_t currentLen = 0;
    if (maxlen == 0) {maxlen = outlen;}
    currentLen = ret.length();
    if (currentLen > maxlen) {
        ret = ret.substring(currentLen - maxlen);
    }
    else {
        while (currentLen < outlen) {
            ret = " " + ret;
            currentLen++;
        }
    }
    return ret;
}

short findIndex(const uint8_t searchVal, const uint8_t* table, const unsigned short tableLen) {
    short index = 0;
    while ( index < (short)tableLen && table[index] != searchVal ) index++;
    return ( index == (short)tableLen ? -1 : index );
}

uint8_t getIncomingSerial(void) {
    uint8_t headerFrame[3] = {0x00,0x00,0x00};
    uint8_t payloadLen = 0;
    uint8_t crc = 0x00;

    if (Serial.readBytes(headerFrame, 3) != 3) {
        // purge input buffer
        while(Serial.available()) {Serial.read();}
        RxErr++;
        appendErrorBuff(E_HEADERLENGTH, headerFrame, 3);
        return 0x00;
    }
    payloadLen = (headerFrame[0] & 0xF0) >> 4;
    crc = headerFrame[0] & 0x0F;
    lastMsgID = headerFrame[1];
    RxBuff[0] = headerFrame[2];
    if (Serial.readBytes(RxBuff+1, payloadLen) != (unsigned short) payloadLen) {
        // purge input buffer
        while(Serial.available()) {Serial.read();}
        appendErrorBuff(E_PAYLOADLENGTH, RxBuff+1, payloadLen);
        RxErr++;
        return 0x00;
    }
    if (genCS(RxBuff, payloadLen+1) != crc) {
        RxErr++;
        appendErrorBuff(E_CRCFAIL, RxBuff, payloadLen+1);
        return 0x00;
    }

    RxStat++;
    return payloadLen+1;
}

void sendAck(bool state = true) {
    uint8_t headerFrame[3] = {0x00, lastMsgID, CONFIGMSG|SYSACKOK};
    if (ackEnabled) {
        if (!state) { headerFrame[2] = CONFIGMSG|SYSACKKO; }
        headerFrame[0] = genCS(headerFrame+2, 1);
        Serial.write(headerFrame, 3);
    }
}

void sendToSerial(const uint8_t* const buffPtr, const unsigned short buffLen) {
    uint8_t frameLen = 0;
    for (unsigned short i = 0; i < buffLen; i += 16) {
        frameLen = min(16,buffLen-i);
        Serial.write(((frameLen-1) << 4) | genCS(buffPtr+i, frameLen));
        Serial.write(lastMsgID);
        Serial.write(buffPtr+i, frameLen);
    }
    TxStat++;
    return;
}

uint8_t genCS(const uint8_t* const data, uint8_t dataLen) {
    uint8_t cs = 0x00;
    // do a 8bits checksum (overflow is ignored by uint8_t type)
    for (uint8_t i=0; i<dataLen; i++) {
        cs += data[i];
    }
    // transform checksum to a 4bits checksum by doing another 4bits checksum
    cs = ((cs&0xF0)>>4)+(cs&0x0F);
    return cs&0x0F;     //overflow is ignored by mask
}

void appendErrorBuff(const uint8_t errorCode, const uint8_t* const additionalInfo, uint8_t infoLen) {
    // warn the host about the error
    errCount++;
    sendAck(false);
    if (errBuffIndex+infoLen+4>=ERRORBUFFLEN) {
        errBuffLen = errBuffIndex;
        errBuffIndex = 0; //recycle the error buffer
    }
    errorBuff[errBuffIndex] = errorCode;
    errBuffIndex++;
    errorBuff[errBuffIndex] = lastMsgID;
    errBuffIndex++;
    if (additionalInfo != NULL && infoLen > 0) {
        strncpy((char*)errorBuff+errBuffIndex, (char*)additionalInfo, infoLen);
        errBuffIndex += infoLen;
    }
    errorBuff[errBuffIndex] = 0x0D;
    errBuffIndex++;
    errorBuff[errBuffIndex] = 0x0A;
    errBuffIndex++;
    errBuffLen = max(errBuffLen, errBuffIndex);
    return;
}

void sendErrorBuff(void) {
    // sendAck(false);
    if (errBuffLen > errBuffIndex) {
        sendToSerial(errorBuff+errBuffIndex, errBuffLen-errBuffIndex);
    }
    sendToSerial(errorBuff, errBuffLen);
}

short checkPinAndUpdate(const uint8_t pin, const uint8_t* const table, const unsigned short tableLen, uint8_t* const assignationList, const uint8_t pinUsage) {
    short index = checkPin(pin, table, tableLen);
    if (index >= 0) {
        if (assignationList[index]) { appendErrorBuff(W_PINALREADYUSED, &pin, 1); }
        assignationList[index] = pinUsage;
    }
    return index;
}

short checkPinAndMode(const uint8_t pin, const uint8_t* const table, const unsigned short tableLen, const uint8_t* const assignationList, const uint8_t pinUsage) {
    short index = checkPin(pin, table, tableLen);
    if (index >= 0) {
        if ((assignationList[index]&pinUsage) != pinUsage) { index = -1; }
    }
    return index;
}

short checkPin(const uint8_t pin, const uint8_t* const table, const unsigned short tableLen) {
    short index = findIndex(pin, table, tableLen);
    if (index < 0) { appendErrorBuff(E_UKNOWNPIN, &pin, 1); }
    return index;
}

// ######################## LCD FUNCTION ########################

#ifdef LCDUSED
void errorDisplayLayout(void) {
    char tmp[2];
    uint8_t index = errBuffIndex;
    uint8_t fromIndex = 0;
    uint8_t len = 0;
    if (displayMode != ERRORDISPLAYMODE) {
        // clear the display
        lcd.clear();
        lcd.write("Debug data:");
        displayMode = ERRORDISPLAYMODE;
    }
    lcd.setCursor(12, 0);
    lcd.print(formatIntStrLen(errCount,3));
    lcd.setCursor(0, 1);
    if (errBuffLen == 0) {
        lcd.write("  Buffer empty  ");
    }
    else {
        index--;

        while(true) {   // parse the errorBuff to find whole msgs which can be displayed on lcd display.

            if (index>0 && errorBuff[index-1]==0x0D && errorBuff[index]==0x0A) {
                // end of error msg detected (a msg ends by CRLF, 0x0D 0x0A).
                fromIndex = index+1;    // save start index of following message.
                index--;    // 2 bytes was processed instead of 1, decrement index one more time.
                len++;
            }
            else {
                len += 2;  // nothing has broken the loop, so count this char (which will use 2 chars, when displayed in hex)...
            }

            // before exiting loop: check if the current byte is the msg start, by check the 2 previous bytes (CRLF)
            if (len>=16) {   // Display allows 16 chars.
                if (index>1 && errorBuff[index-2]==0x0D && errorBuff[index-1]==0x0A) {fromIndex = index;}
                break; // exiting case: 16 bytes was processed
            }

            // updating index.
            if (index == 0) {
                if(errBuffLen != errBuffIndex){index = errBuffLen - 1;}     // that's mean error buffer was recycled, so count from the end of buffer
                else {
                    fromIndex = 0;
                    break;  // exiting case: whole error buffer was processed or is empty.
                }
            }
            else { index--; }

        }

        index = fromIndex;
        len = 0;

        // All error msg end by CRLF, which is replaced by a space.
        while(len<(16-1) && index!=errBuffIndex) { // we need at least 2 chars available on display
            if (index >= errBuffLen) {index = 0;}
            if (index+1<errBuffLen && errorBuff[index]==0x0D && errorBuff[index+1]==0x0A) {
                lcd.print(" ");
                len++;
                index += 2;
            }
            else {
                sprintf(tmp, "%.2x", errorBuff[index]);
                lcd.print(tmp);
                len += 2;
                index++;
            }
        }
    }
}

void configDisplayLayout(void) {
    uint8_t index = 0;

    if(!configMode) {ioDisplayLayout(); return;}

    if (displayMode != CONFIGDISPLAYMODE) {
        // clear the display
        lcd.clear();
        lcd.write("ConfigIO: ");
        displayMode = CONFIGDISPLAYMODE;
    }
    for (index=0; index<6; index++) {
        if (index==AINNUM) {break;}
        lcd.setCursor(10+index, 0);
        if (AINAssignation[index]&AINMODE) { lcd.print("A"); }
        else { lcd.print("U"); }
    }
    for(index=0; index<16; index++) {
        if (index==IONUM) {break;}
        lcd.setCursor(index, 1);
        switch(IOAssignation[index]) {
            case INMODE: lcd.print("I"); break;
            case OUTMODE: lcd.print("O"); break;
            case PWMMODE: lcd.print("P"); break;
            case SERVOMODE: lcd.print("S"); break;
            case UNUSEDPIN: lcd.print("U"); break;
            default: lcd.print("-");
        }
    }
}

void ioDisplayLayout(void) {
    uint8_t index = 0;

    if(configMode) {configDisplayLayout(); return;}

    if (displayMode != IODISPLAYMODE) {
        // clear the display
        lcd.clear();
        lcd.write("IO State: ");
        displayMode = IODISPLAYMODE;
    }
    for (index=0; index<6; index++) {
        if (index==AINNUM) {break;}
        lcd.setCursor(10+index, 0);
        if (AINAssignation[index]&AINMODE) { lcd.print((char)((AINState[index]>>2)+0x30)); }
        else { lcd.print("-"); }
    }
    for(index=0; index<16; index++) {
        if (index==IONUM) {break;}
        lcd.setCursor(index, 1);
        if(IOAssignation[index]&(INMODE|OUTMODE)) {
            lcd.print(IOState&(1<<index)?"1":"0");
        }
        else if(IOAssignation[index] != UNUSEDPIN) {
            lcd.print("X");
        }
        else {
            lcd.print("-");
        }
    }
}

void serialDisplayLayout(void) {
    if (displayMode != SERIALDISPLAYMODE) {
        // clear the display
        lcd.clear();
        lcd.write("Serial: #");
        lcd.setCursor(0, 1);
        lcd.write("R    /T     E");
        displayMode = SERIALDISPLAYMODE;
    }
    lcd.setCursor(9, 0);
    lcd.print(formatIntStrLen(lastMsgID,5));
    lcd.setCursor(1, 1);
    lcd.print(formatIntStrLen(RxStat,4));
    lcd.setCursor(7, 1);
    lcd.print(formatIntStrLen(TxStat,4));
    lcd.setCursor(13, 1);
    lcd.print(formatIntStrLen(RxErr,3));
}

void homeDisplayLayout(void) {
    if (displayMode != HOMEDISPLAYMODE) {
        // clear the display
        lcd.clear();
        lcd.write("     Hello     ");
        lcd.setCursor(0, 1);
        lcd.write("I'm futurakart!");
        displayMode = HOMEDISPLAYMODE;
    }
}

void hostDisplayLayout(void) {
    if (displayMode != HOSTDISPLAYMODE) {
        // clear the display
        lcd.clear();
        lcd.write(" Host message:");
        displayMode = HOSTDISPLAYMODE;
    }
    lcd.setCursor(0, 1);
    lcd.print((char*)LCDBuff);
}

void refreshDisplay(uint8_t switchDisplay = 0x00) {
    if (!switchDisplay) {
        switchDisplay = displayMode;
#ifdef UIBUTTONINDEX
        if (digitalRead(IOList[UIBUTTONINDEX]) == UIBUTTONHIGHSTATE) {
            if (displayMode == DISPLAYMODENUM) {switchDisplay = 0x01;}
            else {switchDisplay++;}
        }
#endif
    }
    switch(switchDisplay) {
        case HOMEDISPLAYMODE: homeDisplayLayout(); break;
        case HOSTDISPLAYMODE: hostDisplayLayout(); break;
        case ERRORDISPLAYMODE: errorDisplayLayout(); break;
        case SERIALDISPLAYMODE: serialDisplayLayout(); break;
        case CONFIGDISPLAYMODE: configDisplayLayout(); break;
        case IODISPLAYMODE: ioDisplayLayout(); break;
        default: break;
    }
}
#endif

// ######################## CONFIG FUNCTIONS ########################

void processMessage(const uint8_t action, const uint8_t* const data, const uint8_t dataLen) {
    uint8_t command = action&ACTIONMASK;
    uint8_t pin = action&PINMASK;
#ifdef LEDSTATUSINDEX
    if (pin == IOList[LEDSTATUSINDEX] && command != CONFIGMSG && command != AX12MSG) {
        appendErrorBuff(W_LEDSTATUSUSED, &action, 1);
        return;
    }
#endif
#ifdef UIBUTTONINDEX
    if (pin == IOList[UIBUTTONINDEX] && command != CONFIGMSG && command != AX12MSG) {
        appendErrorBuff(W_UIBUTTONUSED, &action, 1);
        return;
    }
#endif
    if (configMode) {processConfigMessage(command, pin, data, dataLen);}
    else {processStdMessage(command, pin, data, dataLen);}
    return;
}

void processConfigMessage(const uint8_t command, const uint8_t pin, const uint8_t* const data, const uint8_t dataLen) {
    switch (command) {
        case INMSG:
            if (dataLen == 0) {setIn(pin, 0);}
            else if (dataLen == 1)  {setIn(pin, data[0]);}
            else {appendErrorBuff(W_UKNOWNPARAMETER, data, dataLen);}
            break;
        case OUTMSG:
            if (dataLen == 0) {setOut(pin);}
            else {appendErrorBuff(W_UKNOWNPARAMETER, data, dataLen);}
            break;
        case SERVOMSG:
            if (dataLen == 0) {setServo(pin, 0, 0);}
            else if (dataLen == 2)  {setServo(pin, data[0], data[1]);}
            else {appendErrorBuff(W_UKNOWNPARAMETER, data, dataLen);}
            break;
        case CONFIGMSG:
            if (dataLen == 0) {setConfig(pin);}
            else {appendErrorBuff(W_UKNOWNPARAMETER, data, dataLen);}
            break;
        default :
            appendErrorBuff(W_UKNOWNCOMMAND, &command, 1);
    }
    return;
}

void setIn(const uint8_t pin, const uint8_t threshold) {
    short index = 0;
    if (pin < IONUM) {
        index = checkPinAndUpdate(pin, IOList, IONUM, IOAssignation, threshold?(INMODE|INTHRESHOLD):INMODE);
        // pin error already processed by checkPinAndUpdate function
        if (index >= 0) {
            // pinMode((unsigned short)IOList[index], INPUT);
            pinMode((unsigned short)IOList[index], INPUT_PULLUP);
        }
    }
    else {
        index = checkPinAndUpdate(pin-IONUM, AINList, AINNUM, AINAssignation, threshold?(AINMODE|INTHRESHOLD):AINMODE);
        // pin error already processed by checkPinAndUpdate function
        // Nothing to do to configure AIN pins
        if (index >= 0 && threshold) {
            AINThreshold[index] = threshold;
        }
    }
    return;
}

void setOut(const uint8_t pin) {
    short index = checkPinAndUpdate(pin, IOList, IONUM, IOAssignation, OUTMODE);
    if (index >= 0) {
        pinMode((unsigned short)IOList[index], OUTPUT);
    }
    return;
}

void setServo(const uint8_t pin, const uint8_t minPos, const uint8_t maxPos) {
    short index = findIndex(pin, PWMList, PWMNUM);
    if (index < 0) {
        return;
    }
    checkPinAndUpdate(pin, IOList, IONUM, IOAssignation, SERVOMODE);
    if (minPos != maxPos) {
        if (minPos <= (uint8_t)180 && maxPos <= (uint8_t)180) {
            minMaxServo[index] = ((uint16_t)minPos<<8)|(uint16_t)maxPos;
        }
        else {
            appendErrorBuff(W_WRONGSERVOPARAM, &minPos, 1);
            appendErrorBuff(W_WRONGSERVOPARAM, &maxPos, 1);
        }
    }
    // servoList[index].attach enables the servo. Servo enabling is requested on set position.
}

void setConfig(const uint8_t pin) {
    switch(pin) {
        case SYSRETRIEVEERR:
            sendErrorBuff();    // retrieve error buffer
            break;
        case SYSACKTGL:
            ackEnabled = ~ackEnabled;   // enable/disable acknowledge on serial command
            break;
        case SYSWARNERRTGL:
            warnErr = ~warnErr;
            break;
        case SYSCONFEXIT:
            configMode = false;   // exit config mode
            break;
        case SYSRSTSTATS:
            errBuffLen = 0;     // reset err buff
            errBuffIndex = 0;
            RxErr = 0;          // reset stats
            RxStat = 0;
            TxStat = 0;
            errCount = 0;
            break;
        case SYSREBOOT:
            resetFunc();   // reset board
            break;
        case SYSHALT:
            teardone();
            haltFunc();   // halt board
            break;
        default:
            ;
            break;
    }
    return;
}

// ######################## BASE FUNCTIONS ########################

void processStdMessage(const uint8_t command, const uint8_t pin, const uint8_t* const data, const uint8_t dataLen) {
    switch (command) {
        case INMSG:
            if (dataLen == 0) {processIn(pin, false);}
            else if (dataLen == 1)  {processIn(pin, data[0]==0xFF);}
            else {appendErrorBuff(W_UKNOWNPARAMETER, data, dataLen);}
            break;
        case OUTMSG:
            if (dataLen == 1) {processOut(pin, (data[0]&0x0F)!=0x00, (data[0]&0xF0)==0xF0);}
            else {appendErrorBuff(W_UKNOWNPARAMETER, data, dataLen);}
            break;
        case SERVOMSG:
            if (dataLen == 0) {processServo(pin, false, 0);}
            else if (dataLen == 1)  {processServo(pin, true, data[0]);}
            else {appendErrorBuff(W_UKNOWNPARAMETER, data, dataLen);}
            break;
        case I2CMSG:
            if (pin == I2CGATEWAY) {appendErrorBuff(W_DEBUG, 0, 0);}    // ToDo: I2C gateway
#ifdef LCDUSED
            else if (pin==I2CLCDCOLOR && dataLen==3) {processLCDColor(data);}
            else if (pin==I2CLCDMSG && dataLen!=0) {processLCDmsg(data, dataLen);}
#endif
            else {appendErrorBuff(W_UKNOWNPARAMETER, 0, 0);}
            break;
        case CONFIGMSG:
            if (dataLen == 0) {processConfig(pin);}
            else {appendErrorBuff(W_UKNOWNPARAMETER, data, dataLen);}
            break;
        default :
            appendErrorBuff(W_UKNOWNCOMMAND, &command, 1);
    }
    return;
}

void processIn(const uint8_t pin, const bool allPins) {
    uint8_t buffLen = 0x01;
    short index = 0;
    if (allPins) {
        TxBuff[0] = INMSG|PINMASK;
        for (index=0; index<IONUM; index++) {
            if (buffLen < TXBUFFLEN && IOAssignation[index] == INMODE) {
                TxBuff[buffLen] = digitalRead((unsigned short)IOList[index]) ? 0x01 : 0x00;
                buffLen++;
            }
        }
        for (index=0; index<AINNUM; index++) {
            if (buffLen < TXBUFFLEN && AINAssignation[index] == AINMODE) {
                TxBuff[buffLen] = (uint8_t)(analogRead((unsigned short)AINList[index])>>2);
                buffLen++;
            }
        }
        sendToSerial(TxBuff, buffLen);
    }
    else {
        TxBuff[0] = INMSG|pin;
        if (pin < IONUM) {
            index = checkPinAndMode(pin, IOList, IONUM, IOAssignation, INMODE);
            if (index >=0) {
                TxBuff[1] = digitalRead((unsigned short)IOList[index]) ? 0x01 : 0x00;
                sendToSerial(TxBuff, 2);
            }
        }
        else {
            index = checkPinAndMode(pin-IONUM, AINList, AINNUM, AINAssignation, AINMODE);
            if (index >=0) {
                TxBuff[1] = (uint8_t)(analogRead((unsigned short)AINList[index])>>2);
                sendToSerial(TxBuff, 2);
            }
        }
    }
    return;
}

void processOut(const uint8_t pin, const bool state, const bool allPins) {
    short index = 0;
    if (allPins) {
        for (index=0; index<IONUM; index++) {
            if (IOAssignation[index]&OUTMODE) {
                digitalWrite((unsigned short)IOList[index], state);
            }
        }
    }
    else {
        index = checkPinAndMode(pin, IOList, IONUM, IOAssignation, OUTMODE);
        if (index >=0) {
            digitalWrite((unsigned short)IOList[index], state);
        }
    }
    return;
}

void processServo(const uint8_t pin, const bool enable, const uint8_t pos) {
    unsigned short val = pos;
    short index = findIndex(pin, PWMList, PWMNUM);
    if (index < 0) {
        return;
    }
    if (checkPinAndMode(pin, IOList, IONUM, IOAssignation, SERVOMODE) >=0) {
        if (enable) {
            if (val <= (uint8_t)180) {
                if (!servoList[index].attached()) {
                    if (servoList[index].attach((unsigned short)pin) == 0) {
                        appendErrorBuff(E_SERVOINIT, &pin, 1);
                        return;
                    }
                }
                if (minMaxServo[index] != (uint16_t)0) {
                    val = min((unsigned short)(minMaxServo[index]&0x00FF),max((unsigned short)((minMaxServo[index]&0xFF00)>>8),val));
                }
                servoList[index].write(val);
            }
            else {
                appendErrorBuff(W_WRONGSERVOPARAM, (const uint8_t*) &val, 1);
            }
        }
        else {
            servoList[index].detach();
        }
    }
    return;
}

#ifdef LCDUSED
void processLCDmsg (const uint8_t* const msg, const uint8_t dataLen) {
    uint8_t index = dataLen;
    strncpy((char*)LCDBuff, (char*)msg, dataLen);
    while (index<16) {
        LCDBuff[index] = 0x20;  // space character
        index++;
    }
}

void processLCDColor (const uint8_t* const colorFrame) {
    lcd.setRGB(colorFrame[0], colorFrame[1], colorFrame[2]);
}
#endif

void releaseAllServos (void) {
    for (uint8_t index=0; index<PWMNUM; index++) {
        processServo(PWMList[index], false, 0);
    }
    return;
}

void processConfig(const uint8_t pin) {
    switch(pin) {
        case SYSRETRIEVEERR:
            ;   // retrieve error buffer
            break;
        case SYSGETVERSION:
            strncpy((char*)TxBuff, VERSION, 3);
            sendToSerial(TxBuff,3);
            break;
        case SYSCONFENTER:
            configMode = true;   // enter in config mode
            break;
        default:
            ;
            break;
    }
    return;
}

void refreshInputs(void) {
    uint8_t index = 0;
    if (!configMode) {
        for(index=0; index<IONUM; index++) {
            if (IOAssignation[index]&INMODE) {
                if (digitalRead(IOList[index])) {IOState |= (1<<index);}
                else {IOState &= ~(1<<index);}
            }
        }
        for (index=0; index<AINNUM; index++) {
            if (AINAssignation[index]&AINMODE) {
                AINState[index] = (uint8_t)(analogRead((unsigned short)AINList[index])>>2);
            }
        }
   }
}

bool monitorThreshold(void) {
    bool res = false;
    uint8_t index = 0;
    uint8_t readVal = 0;
    if (!configMode) {
        for(index=0; index>IONUM; index++) {
            if ((IOAssignation[index]&(INMODE|INTHRESHOLD)) == (INMODE|INTHRESHOLD)) {
                if (((uint8_t)digitalRead(IOList[index])) != (IOState&(1<<index))) {
                    processIn(IOList[index], false);
                    res = true;
                }
            }
        }
        for (index=0; index<AINNUM; index++) {
            if ((AINAssignation[index]&(AINMODE|INTHRESHOLD)) == (AINMODE|INTHRESHOLD)) {
                readVal = (uint8_t)(analogRead((unsigned short)AINList[index])>>2);
                if ((AINState[index]<AINThreshold[index]) == (readVal>=AINThreshold[index])) {
                    processIn(AINList[index], false);
                    res = true;
                }
            }
        }
    }
    return res;
}

// ######################## MAIN FUNCTION ########################

void loop() {
    // put your main code here, to run repeatedly:
    uint8_t dataLen = 0;

    // loop refreshes @ 10Hz
    refreshInputs();

    if (Serial.available()) {
        dataLen = getIncomingSerial();
        if(dataLen) {
            sendAck();
            processMessage(RxBuff[0], RxBuff+1, dataLen-1);
        }
    }

    if ((loopCounter&0x07) == 0x00) {   //2.5Hz refresh
#ifdef LCDUSED
        if (displayMode == 0x00) {refreshDisplay(HOMEDISPLAYMODE);}
        else {refreshDisplay();}
#endif
#ifdef LEDSTATUSINDEX
        digitalWrite((unsigned short)IOList[LEDSTATUSINDEX], loopCounter&0x08);
#endif
    }

    // minLoopFreeTime = min(minLoopFreeTime, (nextLoopTime - millis()));
    // appendErrorBuff(I_LOOPLOAD, &minLoopFreeTime, 1);

    while(true) {
        // do monitoring and break the loop if necessary.
        if (monitorThreshold()) {break;}        // do checking @ max frequency.
        if (Serial.available()) {break;}        // If a incoming message is pending.
        if (nextLoopTime < millis()) {break;}   // timeout to perform whole loop.
    }
    nextLoopTime = millis() + LOOPPERIOD;
    if (loopCounter == 0x0F) {loopCounter = 0x00;}
    else {loopCounter++;}

}
