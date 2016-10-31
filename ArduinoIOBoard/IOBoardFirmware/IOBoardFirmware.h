//
// File: IOBoard.cpp
// Description: IO Board firmware
//
// History
// Local versions
// 06/05/2016 v0.3  Fix readINPin issue.
// 02/10/2016 v0.4  Fix servo issue.
// 28/10/2016 v0.5  Refactoring for Arduino 1.7.x.
//
#ifndef __IOBoardFirmware__

#define __IOBoardFirmware__

// include the library code:
#include <Arduino.h>
#include <string.h>
#include <Servo.h>
#include <Wire.h>
// #include <avr/wdt.h>    //WDT not supported on Arduino101 (wdt_disable, wdt_enable(WDTO_2S), wdt_reset not supported)
#include <rgb_lcd.h>
//#include <DynamixelSerial1.h>

#define VERSION "0.5"

// define board capabilities

// #define ARDUINO101
#define ARDUINOUNO
// #define ARDUINOM0

#ifdef ARDUINO101
// define board ressources & assignations
#define IONUM 14
#define AINNUM 6
#define PWMNUM 4
const uint8_t IOList[IONUM] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D};
uint16_t IOState = 0;
const uint8_t AINList[AINNUM] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t AINState[AINNUM] = {0};
const uint8_t PWMList[PWMNUM] = {0x03, 0x05, 0x06, 0x09};
Servo servoList[PWMNUM];
#define UIBUTTONINDEX    12
#define UIBUTTONHIGHSTATE 0
#define LEDSTATUSINDEX   13
#define LCDUSED
#endif

#ifdef ARDUINOUNO
//  /!\ Need a 10uF condensator between GND and RESET pin to avoid reset on serial connection
// define board ressources & assignations
#define IONUM 14
#define AINNUM 6
#define PWMNUM 6
const uint8_t IOList[IONUM] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D};
uint16_t IOState = 0;
const uint8_t AINList[AINNUM] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t AINState[AINNUM] = {0};
const uint8_t PWMList[PWMNUM] = {0x03, 0x05, 0x06, 0x09, 0x0A, 0x0B};
Servo servoList[PWMNUM];
#define UIBUTTONINDEX    12
#define UIBUTTONHIGHSTATE 0
#define LEDSTATUSINDEX   13
// #define LCDUSED
#endif


// define flags for supported pin uses
#define UNUSEDPIN   0b00000000
#define INMODE      0b00000001
#define AINMODE     0b00000010
#define INTHRESHOLD 0b00000100
#define OUTMODE     0b00001000
#define PWMMODE     0b00010000
#define SERVOMODE   0b00100000

// ############################## Serial protocol: ##############################
// 1st bype: b7..b4 -> Frame length (3rd frame byte excluded), b3..b0 -> checksum
// 2nd byte: message ID
// from 3rd byte: payload.
// ##############################################################################

// define action messages mask and values
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

// define I2C pin mapping reuse (all are masked by I2CMSG)
#define I2CLCDMSG   0x00
#define I2CLCDCOLOR 0x01
#define I2CGATEWAY  0x10

// define system pin mapping reuse (all are masked by CONFIGMSG)
#define SYSRETRIEVEERR 0x00
#define SYSACKTGL      0x01
#define SYSWARNERRTGL  0x02
#define SYSGETVERSION  0x03
#define SYSCONFENTER   0x05
#define SYSACKOK       0x06
#define SYSACKKO       0x07
#define SYSCONFEXIT    0x0A
#define SYSRSTSTATS    0x0F
#define SYSHALT        0x14
#define SYSREBOOT      0x15

// define Serial parameters
#define USEFULLFRAMELEN 16
#define RXBUFFLEN USEFULLFRAMELEN
#define TXBUFFLEN USEFULLFRAMELEN
#define ERRORBUFFLEN 90

// define error messages
#define E_CRCFAIL         0x01
#define E_HEADERLENGTH    0x02
#define E_PAYLOADLENGTH   0x03
#define E_UKNOWNPIN       0x04
#define E_SERVOINIT       0x05
#define W_DEBUG           0x20
#define W_UKNOWNCOMMAND   0x21
#define W_UKNOWNPARAMETER 0x22
#define W_PINALREADYUSED  0x23
#define W_LEDSTATUSUSED   0x24
#define W_UIBUTTONUSED    0x25
#define W_WRONGSERVOPARAM 0x26
#define I_LOOPLOAD        0x51

// ############################## Error format: ##############################
// 1st bype: errorCode (see above)
// 2nd byte: message ID
// from 3rd byte: payload.
// ##############################################################################

// define LCD parameters
#define DISPLAYMODENUM    0x06
#define HOMEDISPLAYMODE   0x01
#define HOSTDISPLAYMODE   0x02
#define ERRORDISPLAYMODE  0x03
#define SERIALDISPLAYMODE 0x04
#define CONFIGDISPLAYMODE 0x05
#define IODISPLAYMODE     DISPLAYMODENUM

#undef min
#undef max
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

// ######################## SYSTEM FUNCTIONS ########################

void haltFunc(void);

void teardone (void);

// ######################## DATA MANAGEMENT ########################

String formatIntStrLen(const unsigned short, const uint8_t, uint8_t);

short findIndex(const uint8_t, const uint8_t*, const unsigned short);

uint8_t getIncomingSerial(void);

void sendAck(bool);

void sendToSerial(const uint8_t* const, const unsigned short);

uint8_t genCS(const uint8_t* const, uint8_t);

void appendErrorBuff(const uint8_t, const uint8_t* const, uint8_t);

void sendErrorBuff(void);

short checkPinAndUpdate(const uint8_t, const uint8_t* const, const unsigned short, uint8_t* const, const uint8_t);

short checkPinAndMode(const uint8_t, const uint8_t* const, const unsigned short, const uint8_t* const, const uint8_t);

short checkPin(const uint8_t, const uint8_t* const, const unsigned short);

// ######################## LCD FUNCTION ########################

#ifdef LCDUSED
void errorDisplayLayout(void);

void configDisplayLayout(void);

void ioDisplayLayout(void);

void serialDisplayLayout(void);

void homeDisplayLayout(void);

void hostDisplayLayout(void);

void refreshDisplay(uint8_t);
#endif

// ######################## CONFIG FUNCTIONS ########################

void processMessage(const uint8_t, const uint8_t* const, const uint8_t);

void processConfigMessage(const uint8_t, const uint8_t, const uint8_t* const, const uint8_t);

void setIn(const uint8_t, const uint8_t);

void setOut(const uint8_t);

void setServo(const uint8_t, const uint8_t, const uint8_t);

void setConfig(const uint8_t);

// ######################## STD FUNCTIONS ########################

void processStdMessage(const uint8_t, const uint8_t, const uint8_t* const, const uint8_t);

void processIn(const uint8_t, const bool);

void processOut(const uint8_t, const bool, const bool);

void processServo(const uint8_t, const bool, const uint8_t);

#ifdef LCDUSED
void processLCDmsg (const uint8_t* const, const uint8_t);

void processLCDColor (const uint8_t* const);
#endif

void releaseAllServos (void);

void processConfig(const uint8_t);

void refreshInputs(void);

bool monitorThreshold(void);

#endif

