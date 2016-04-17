// include the library code:
#include <string.h>
#include <Servo.h>
#include <Wire.h>
// #include <avr/wdt.h>    //WDT not supported on Arduino101 (wdt_disable, wdt_enable(WDTO_2S), wdt_reset not supported)
#include "rgb_lcd.h"


// define capabilities
#define IONUM 14
#define AINNUM 6
#define PWMNUM 4
bool configMode = false;
const uint8_t IOList[IONUM] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D};
uint16_t IOState = 0;
const uint8_t AINList[AINNUM] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t AINState[AINNUM] = {0};
const uint8_t PWMList[PWMNUM] = {0x03, 0x05, 0x06, 0x09};
Servo servoList[PWMNUM];
#define LEDSTATUSINDEX 13

// define supported pin uses
#define UNUSEDPIN   0b00000000
#define INMODE      0b00000001
#define AINMODE     0b00000010
#define INTHRESHOLD 0b00000100
#define OUTMODE     0b00001000
#define PWMMODE     0b00010000
#define SERVOMODE   0b00100000
uint8_t IOAssignation[IONUM] = {0};
uint8_t AINAssignation[AINNUM] = {0};
uint8_t AINThreshold[AINNUM] = {0};
uint16_t minMaxServo[PWMNUM] = {0};

// define action messages
#define ACTIONMASK 0xE0
#define PINMASK    0x1F
#define INMSG      0x00
#define OUTMSG     0x20
#define SERVOMSG   0x40
#define PWMMSG     0x60
#define AX12MSG    0x80
#define I2CMSG     0xA0
#define UARTMSG    0xC0
#define CONFIGMSG  ACTIONMASK
#define USEFULLFRAMELEN 16
#define RXBUFFLEN USEFULLFRAMELEN
#define TXBUFFLEN USEFULLFRAMELEN
#define ERRORBUFFLEN 1000

unsigned short RxStat = 0;
unsigned short RxErr = 0;
unsigned short TxStat = 0;
uint8_t lastMsgID = 0;
uint8_t* RxBuff = (uint8_t*)calloc(RXBUFFLEN, sizeof(uint8_t)); // max serial data payload len + 1
uint8_t* TxBuff = (uint8_t*)calloc(TXBUFFLEN, sizeof(uint8_t));
uint8_t* errorBuff = (uint8_t*)calloc(ERRORBUFFLEN, sizeof(uint8_t));
unsigned short errBuffLen = 0;
unsigned short errBuffIndex = 0;

// define error message
#define E_CRCFAIL         0x01
#define I_MSGID           0x02
#define E_HEADERLENGTH    0x03
#define E_PAYLOADLENGTH   0x04
#define W_UKNOWNCOMMAND   0x11
#define W_UKNOWNPARAMETER 0x12
#define E_UKNOWNPIN       0x21
#define W_PINALREADYUSED  0x22
#define W_LEDSTATUSUSED   0x23
#define E_SERVOINIT       0x24
#define W_WRONGSERVOPARAM 0x25

rgb_lcd lcd;
#define HOMEDISPLAYMODE 0x01
#define DEBUGDISPLAYMODE 0x02
#define SERIALDISPLAYMODE 0x03
#define CONFIGDISPLAYMODE 0x04
#define IODISPLAYMODE 0x05
uint8_t displayMode = 0x00;

#define LOOPPERIOD 100      //ms
unsigned long nextLoopTime = 0;
uint8_t loopCounter = 0;

// TODO a watchdog...
void(* haltFunc) (void) = 0;//declare reset function at address 0 == goto 0 address.

void setup(void) {
    // put your setup code here, to run once:

    // set pin for LED status
    #ifdef LEDSTATUSINDEX
    IOAssignation[LEDSTATUSINDEX] = OUTMODE;
    pinMode((unsigned short)IOList[LEDSTATUSINDEX], OUTPUT);
    digitalWrite((unsigned short)IOList[LEDSTATUSINDEX], HIGH);
    #endif

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    lcd.setRGB(255, 255, 255);
    // initialize the serial communications:
    lcd.clear();
    lcd.write("Boot on-going...");
    lcd.setCursor(0, 1);
    lcd.write("Wainting serial ");

    Serial.begin(115200);
    while (!Serial && millis() < 10000) {
        delay(100); // wait for serial port to connect. Needed for native USB port only
    }
    if (Serial) {
        lcd.setCursor(0, 1);
        lcd.write("Serial ready    ");
    }
    else {
        lcd.setRGB(255, 0, 0);
        lcd.setCursor(0, 1);
        lcd.write("Serial failure  ");
        delay(5000);
        haltFunc();
    }
    delay(500);
    Serial.println("Ready");
    delay(500);
}

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

void debugDisplayLayout(void) {
    if (displayMode != DEBUGDISPLAYMODE) {
        // clear the display
        lcd.clear();
        lcd.write("Debug data:");
        lcd.setCursor(0, 1);
        lcd.write(" ");
        displayMode = DEBUGDISPLAYMODE;
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
            case SERVOMODE: lcd.print("D"); break;
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
        if (AINAssignation[index]&AINMODE) { lcd.print((char)(AINState[index]>>2)+0x30); }
        else { lcd.print("-"); }
    }
    for(index=0; index<16; index++) {
        if (index==IONUM) {break;}
        lcd.setCursor(index, 1);
        if(IOAssignation[index]&(INMODE|OUTMODE)) {
            lcd.print(IOState&(1<<index)?"1":"0");
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
    // clear the display
    lcd.clear();

    lcd.write("Welcome         ");
    lcd.setCursor(0, 1);
    lcd.write("                ");
}

void refreshDisplay(uint8_t switchDisplay = 0x00) {
    if (!switchDisplay) {switchDisplay = displayMode;}
    switch(switchDisplay) {
        case HOMEDISPLAYMODE: homeDisplayLayout(); break;
        case DEBUGDISPLAYMODE: debugDisplayLayout(); break;
        case SERIALDISPLAYMODE: serialDisplayLayout(); break;
        case CONFIGDISPLAYMODE: configDisplayLayout(); break;
        case IODISPLAYMODE: ioDisplayLayout(); break;
        default: break;
    }
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
    if (genCrc(RxBuff, payloadLen+1) != crc) {
        RxErr++;
        appendErrorBuff(E_CRCFAIL, RxBuff+1, payloadLen);
        return 0x00;
    }

    RxStat++;
    return payloadLen+1;
}

void sendAck(void) {
    uint8_t headerFrame[3] = {0x00, lastMsgID, 0x00};
    Serial.write(headerFrame, 3);
}

void sendToSerial(const uint8_t* const buffPtr, const unsigned short buffLen) {
    uint8_t frameLen = 0;
    for (unsigned short i = 0; i < buffLen; i += 16) {
        frameLen = min(16,buffLen-i);
        Serial.write(((frameLen-1) << 4) | genCrc(buffPtr+i, frameLen));
        Serial.write(lastMsgID);
        Serial.write(buffPtr+i, frameLen);
    }
    TxStat++;
    return;
}

uint8_t genCrc(const uint8_t* const data, uint8_t dataLen) {
    uint8_t crc = 0x00;
    for (uint8_t i=0; i<dataLen; i++) {
        crc += data[i];
    }
    return crc & 0x0F;
}

void appendErrorBuff(const uint8_t errorCode, const uint8_t* const additionalInfo, uint8_t infoLen) {
    if (errBuffIndex+infoLen+4>=ERRORBUFFLEN) {errBuffIndex=0;} //recycle the error buffer
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

void processMessage(const uint8_t action, const uint8_t* const data, const uint8_t dataLen) {
    uint8_t command = action&ACTIONMASK;
    uint8_t pin = action&PINMASK;
#ifdef LEDSTATUSINDEX
    if (pin == IOList[LEDSTATUSINDEX] && command != CONFIGMSG) {
        appendErrorBuff(W_LEDSTATUSUSED, &action, 1);
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

void setIn(const uint8_t pin, const uint8_t threshold) {
    short index = checkPinAndUpdate(pin, IOList, IONUM, IOAssignation, threshold?(INMODE|INTHRESHOLD):INMODE);
    if (index < 0) {
        index = checkPinAndUpdate(pin-IONUM, AINList, AINNUM, AINAssignation, threshold?(AINMODE|INTHRESHOLD):AINMODE);
        // pin error already processed by checkPinAndUpdate function
        if (index >= 0 && threshold) {
            AINThreshold[index] = threshold;
        }
    }
    else {
        pinMode((unsigned short)IOList[index], INPUT);
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
    index = checkPinAndUpdate(pin, IOList, IONUM, IOAssignation, SERVOMODE);
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
        case 0x00:
            ;   // retrieve error buffer
            break;
        case 0x0A:
            configMode = false;   // exit config mode
            break;
        case 0x0F:
            ;   // reset stats
            break;
        case 0x15:
            haltFunc();   // reset / halt board
            break;
        default:
            ;
            break;
    }
    return;
}

void processStdMessage(const uint8_t command, const uint8_t pin, const uint8_t* const data, const uint8_t dataLen) {
    switch (command) {
        case INMSG:
            if (dataLen == 0) {processIn(pin, false);}
            else if (dataLen == 1)  {processIn(pin, data[0]==0x00);}
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
    uint8_t buffLen = 0x00;
    uint8_t readVal = 0x00;
    short index = 0;
    if (allPins) {
        for (index=0; index<IONUM; index++) {
            if (buffLen < TXBUFFLEN && IOList[index] == INMODE) {
                TxBuff[buffLen] = digitalRead((unsigned short)IOList[index]) ? 0x01 : 0x00;
                buffLen++;
            }
        }
        for (index=0; index<AINNUM; index++) {
            if (buffLen < TXBUFFLEN && AINList[index] == AINMODE) {
                TxBuff[buffLen] = (uint8_t)(analogRead((unsigned short)AINList[index])>>2);
                buffLen++;
            }
        }
        sendToSerial(TxBuff,buffLen);
    }
    else {
        if (pin < IONUM) {
            index = checkPinAndMode(pin, IOList, IONUM, IOAssignation, INMODE);
            if (index >=0) {
                readVal = digitalRead((unsigned short)IOList[index]) ? 0x01 : 0x00;
                sendToSerial(&readVal, 1);
            }
        }
        else {
            index = checkPinAndMode(pin-IONUM, AINList, AINNUM, AINAssignation, AINMODE);
            if (index >=0) {
                readVal = (uint8_t)(analogRead((unsigned short)AINList[index])>>2);
                sendToSerial(&readVal, 1);
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
    unsigned short minPulseWidth=0, maxPulseWidth=0;
    short index = findIndex(pin, PWMList, PWMNUM);
    if (index < 0) {
        return;
    }
    index = checkPinAndMode(pin, IOList, IONUM, IOAssignation, SERVOMODE);
    if (index >=0) {
        if (enable) {
            if (pos <= (uint8_t)180) {
                if (!servoList[index].attached()) {
                    if (minMaxServo[index] != (uint16_t)0) {
                        minPulseWidth = map((unsigned short)((minMaxServo[index]&0xFF00)>>8), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
                        maxPulseWidth = map((unsigned short)(minMaxServo[index]&0x00FF), 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
                        if (servoList[index].attach((unsigned short)pin, minPulseWidth, maxPulseWidth) == 0) {
                            appendErrorBuff(E_SERVOINIT, &pin, 1);
                            return;
                        }
                    }
                    else {
                        if (servoList[index].attach((unsigned short)pin) == 0) {
                            appendErrorBuff(E_SERVOINIT, &pin, 1);
                            return;
                        }
                    }
                }
                servoList[index].write((unsigned short)pos);
            }
            else {
                appendErrorBuff(W_WRONGSERVOPARAM, &pos, 1);
            }
        }
        else {
            servoList[index].detach();
        }
    }
    return;
}

void processConfig(const uint8_t pin) {
    switch(pin) {
        case 0x00:
            ;   // retrieve error buffer
            break;
        case 0x05:
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
    for(index=0; index>IONUM; index++) {
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

bool monitorThreshold(void) {
    bool res = false;
    uint8_t index = 0;
    uint8_t readVal = 0;
    for(index=0; index>IONUM; index++) {
        if ((IOAssignation[index]&(INMODE|INTHRESHOLD)) == (INMODE|INTHRESHOLD)) {
            if (((uint8_t)digitalRead(IOList[index])) != (IOState&(1<<index))) {
                processIn(IOList[index], false);
                res=true;
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
    return res;
}

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

    if ((loopCounter&0x03) == 0x00) {   //2.5Hz refresh
        if (displayMode == 0x00) {refreshDisplay(IODISPLAYMODE);}
        else {refreshDisplay();}
#ifdef LEDSTATUSINDEX
        digitalWrite((unsigned short)IOList[LEDSTATUSINDEX], loopCounter&0x04);
#endif
    }

    while(nextLoopTime > millis()) {
        // do monitoring and break the loop if necessary
        if (monitorThreshold()) {break;}    // do checking @ max frequency
    }
    nextLoopTime = millis() + LOOPPERIOD;
    if (loopCounter == 0x07) {loopCounter = 0x00;}
    else {loopCounter++;}


}
