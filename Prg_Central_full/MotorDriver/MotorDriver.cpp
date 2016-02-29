/**********************************************************************************************
 * Motor Driver Library - Version 1.0
 * by Lucas Soubeyrand
 
 * This Library is licensed under Copyright
 
 It just interfaces speed (in pourcentage) and direction (digital 0 or 1) into 
a pwm and digital commands compatible to drive a DFR8 shield from a Nucleo F401
*********************************************************************************************/

#include "MotorDriver.h"


MotorDriver::MotorDriver() : Motor1Vit(MOTOR_1VIT), Motor1Dir(MOTOR_1DIR), Motor2Vit(MOTOR_2VIT), Motor2Dir(MOTOR_2DIR){
    Motor1Vit.period(1.0/20000);
    Motor2Vit.period(1.0/20000);
}

  // *****************************FONCTIONS DE CONTROLE MOTEUR *****************************//

  //Attention, les 2 moteurs ne tournent pas dans le meme sens => Attention:engendre bcp d'érreurs !!!

void MotorDriver::Motor1(float vit)  {     
    vit = vit/255.0;    
    if ( vit >= 0 ) {      
      
      Motor1Dir = 0; 
      Motor1Vit.write(vit);     
           
    }
    else {
      
      Motor1Dir = 1; 
      Motor1Vit.write(-vit); 
    }
  }
  
void MotorDriver::Motor2(float vit)  {
    vit = vit/255.0;
    if ( vit >= 0 ) {        
      Motor2Dir = 0; 
      Motor2Vit.write(vit);     
    }
    else {    
      Motor2Dir = 1; 
      Motor2Vit.write(-vit);            
    }
  }
