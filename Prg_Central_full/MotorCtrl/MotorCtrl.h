/**********************************************************************************************
 * Motor Driver Library - Version 1.0
 * by Lucas Soubeyrand and David
 
 * This Library is licensed under Copyleft
 
 It handles low level (position and speed) control of Continuous Current Motors 
 and Alpha/Delta control of a dual motot mobile platform.
 It obviously handles the encoders feedback
*********************************************************************************************/


#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H
 
#include "mbed.h"
#include "USBSerialCom.h"
#include "QEI.h"

#include "MotorDriver.h"
#include "pid.h"
#include "Asserv.h"
#include "Buffer.h"


#define Encoder1_A PC_12//D6//PA_3
#define Encoder1_B PC_10//D7//PA_2

#define Encoder2_A PB_1
#define Encoder2_B PB_2


#define TE 100                     // Temps d'échantiollonnage voulu en 
#define R_WHEEL 0.025                // rayon des roues codeuses en m
#define WHEEL_B 0.188               // écartement des roues codeuses en m
#define ENCODER_RES 600.0          // Nombre de ticks par tour de roue

//29.7 tick/mm



class MotorCtrl {
    
public:    

    //MotorCtrl();  //COnstructor 
    MotorCtrl(USBSerialCom& _comPC);  //COnstructor 
    
    PID PidAngle;
    PID PidDistance;
    float pidAngleOutput;
    float pidDistanceOutput;
    
    void enable(bool is_enabled);
    
    bool DataAvailable();
    void UpdateCmd();
    void Compute();
    void Interrupt_Handler_Encoder();
   // void Control_interrupt();  //call temporal interrupt functions
    void ComputeOdometry();
    void SystemCtrl();
    void ResetCtrl();
    
    void Debug(int orien, int dist);

    double getODO_X();
    double getODO_Y();
    double getODO_Theta();
    
    void setX(float x);
    void setY(float y);
    void setAngle(float Theta);
    void setTarget(float distance, float angle, float finalDistanceSpeed, float finalAngleSpeed, int uuid);
    void setTargetXY(float x, float y, float finalDistanceSpeed, int mode, int uuid); //mode rotation = 1, distance = 2 , both = 3
    void setTargetAngle(float angleAbs, int uuid);
    
    void setTickToAngle(float tickToAngle); 
    void setTickToDistance(float tickToDistance);
    
    double getODO_SPEED_X();
    double getODO_SPEED_Y();
    double getODO_SPEED_Theta();
    
    long getWheelL();
    long getWheelR();
    
  
    float Compute_PID_Angle(float feedbackAspeed, float setpointAspeed);
    float Compute_PID_Linear(float feedbackLspeed, float setpointLspeed);
    
    Asserv distanceCommand;
    Asserv angleCommand;

    
    Buffer <int> angle_buf;
    Buffer <int> distance_buf;
//private:  

    bool isEnabled;

    volatile long encoder1Pos;   // ce sont les 2 entiers ultra important sur lesquels reposent les encodeurs, l'odomètrie et l'asservicement
    volatile long encoder2Pos;   // volatile ne sert à rien^^
    double encoder_position_old1, encoder_position_old2; 
    
    float blockageDistanceTreshold;
    float blockageAngleTreshold;
    
    
    double ODO_X , ODO_Y , ODO_Theta , ODO_khi, ODO_ds; // variable de position initialisées
    double ODO_Theta_Offset;
    double ODO_SPEED_X, ODO_SPEED_Y, ODO_SPEED_Theta; //variables de vitesses
    
    double tickToDistance;
    double tickToAngle;

    double ODO_DELTA_X , ODO_DELTA_Y , ODO_DELTA_Theta;
    
    int commandUUID; // uid for sync between robot status and cmd sent
    int mode_deplacement;// sert à déffinir le mode de déplacement : polaire, linèaire...(ici 1 seul mode)
     
  
    int vitesse_roue_1, vitesse_roue_2, vitesse;
    
   // DigitalOut _pin;
   float pidA, pidT;
   int pidR, pidL;
   

    QEI wheelL;
    QEI wheelR;
    
    int distance;
    int angle;
    
    int previousDistance;
    int previousAngle;
    int deltaAngle;
    int deltaDistance;
    
    int wheelLTick;
    int wheelRTick;
    
    MotorDriver Motors;
    
    USBSerialCom& ComPC;

};
#endif 