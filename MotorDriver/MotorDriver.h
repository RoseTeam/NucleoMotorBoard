#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
 
#include "mbed.h"


#define MOTOR_1VIT PB_10
#define MOTOR_1DIR PA_8

#define MOTOR_2VIT PB_4
#define MOTOR_2DIR PB_5


class MotorDriver {
    
public:  
      // *****************************FONCTIONS DE CONTROLE MOTEUR *****************************//

  //Attention, les 2 moteurs ne tournent pas dans le meme sens => Attention:engendre bcp d'érreurs !!!
  
  MotorDriver(PinName pinMotor1Vit, PinName pinMotor1Dir, PinName pinMotor2Vit, PinName pinMotor2Dir) : Motor1Vit(pinMotor1Vit), Motor1Dir(pinMotor1Dir), Motor2Vit(pinMotor2Vit), Motor2Dir(pinMotor2Dir)
  {      
  };
  
  MotorDriver();
  
  void Motor1(float vit);   
  void Motor2(float vit);
  

private:

    PwmOut Motor1Vit;
    DigitalOut Motor1Dir;

    PwmOut Motor2Vit;
    DigitalOut Motor2Dir;

};

#endif