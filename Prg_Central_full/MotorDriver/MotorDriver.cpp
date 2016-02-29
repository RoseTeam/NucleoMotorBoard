#include "MotorDriver.h"


MotorDriver::MotorDriver() : Motor1Vit(MOTOR_1VIT), Motor1Dir(MOTOR_1DIR), Motor2Vit(MOTOR_2VIT), Motor2Dir(MOTOR_2DIR){
    Motor1Vit.period(1.0/20000);
    Motor2Vit.period(1.0/20000);
}

  // *****************************FONCTIONS DE CONTROLE MOTEUR *****************************//

  //Attention, les 2 moteurs ne tournent pas dans le meme sens => Attention:engendre bcp d'érreurs !!!

void MotorDriver::Motor1(float vit)  {         
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
    if ( vit >= 0 ) {        
      Motor2Dir = 0; 
      Motor2Vit.write(vit);     
    }
    else {    
      Motor2Dir = 1; 
      Motor2Vit.write(-vit);            
    }
  }
