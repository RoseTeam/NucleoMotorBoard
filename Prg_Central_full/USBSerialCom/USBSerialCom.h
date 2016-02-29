#ifndef USB_SERIAL_COM_H
#define USB_SERIAL_COM_H


#include "mbed.h"
//#include "MotorCtrl.h"
//#include "MotorDriver.h"

#define NBR_CHAR_NAV 16

#define KP_POLAR_ANGLE 100
#define KD_POLAR_ANGLE 0.0
#define KI_POLAR_ANGLE 0.0
#define KI_POLAR_ANGLE_SAT 100.0

#define KP_POLAR_LINEAR 600
#define KD_POLAR_LINEAR 0.0
#define KI_POLAR_LINEAR 0.000
#define KI_POLAR_LINEAR_SAT 1.0

#define COM_TIMEOUT 100  // in ms


class USBSerialCom {
    
public:    

    //USBSerialCom(Serial& _pc, MotorCtrl& _asser);  //COnstructor 
    USBSerialCom(Serial& _pc);  //COnstructor 
    
 //   MotorCtrl asser2; 

    

    void serialCallback(); //function to be called when an event occure on the serial bus (basically store the message type and the message byte in an array if relevant)
    int interpretData(); //Function to be called when we received the full packet
    
    void printMetrics(); // print data like odometri or metrics
    void printOdo(); // print data like odometri or metrics
    void printRobotStatus();
    
    void setSStatus(bool value);
    
    int getTtwist();
    int getVtwist();
    bool getSStatus();
    
    bool getUPower() ;
    
    float getKpPL();
    float getKdPL();
    float getKiPL();
    float getKiPLS();
    
    float getKpPA();
    float getKdPA();
    float getKiPA();
    float getKiPAS();
    
    void sendFeedback(long pidL, long pidR, float pidA, float pidT);
    void sendHeartBeat(long data);
    void sendCoeffs();
    
    bool checkTimeOut();
    
private:  

    Timer t_timeout_com;
    
    char inputString[NBR_CHAR_NAV];
    char incomming_message_type;
    char nbr_incom_char;
    long incom_data; 
        
    bool sign;
    
    //commandes/setpoints de l'assrvissement
    long Xorder;
    long Yorder;   
    long Aorder;
    int Lspeed;
    int Rspeed;
    int Ttwist;
    int Vtwist;
    bool SStatus;
    bool UPower;
    
    
    // coefficients d'asservissement pour le polaire
    long KpPL;
    long KdPL;
    long KiPL;
    long KiPLS;
    
    long KpPA;
    long KdPA;
    long KiPA;
    long KiPAS;
    
    bool metricsEnabled;
    bool odoEnabled;
    
    Serial& pc;
   // MotorCtrl& asser;
    
};

#endif
 