#include "SpeedCommand.h"
#include <math.h>

#define abs(x) ((x) < 0 ? - (x) : (x))

SpeedCommand::SpeedCommand()
{
    stepPerMeter = 1000;
    meterPerStep = 1.0/stepPerMeter;
    sampleTime = 4.0/1000;
    blockageTreshold = 0.010;
    blockageValue = 0.0;
    SetSpeed(0);
}

 void SpeedCommand::setThreshold(float threshold){
    blockageTreshold = threshold;     
}

void SpeedCommand::blockageDetector(float actualPosition){
    blockageValue = actualPosition - target;
    if (abs(blockageValue) >= (blockageTreshold * stepPerMeter)){
        target = actualPosition;
        SetSpeed(0);   
    }
}

float SpeedCommand::getBlockageValue(){
   return blockageValue * meterPerStep;
}


void SpeedCommand::ComputeSpeed(){
    float step;
    remainingStep += stepPerSampleTime;
    step = floor(remainingStep);
    target += step;
    remainingStep -= step;
}

void SpeedCommand::SetSampleTime(float p_samplePeriod){
    sampleTime = p_samplePeriod;
    SetSpeed(speed);
}

void SpeedCommand::SetStepPerMeter(float p_stepPerMeter){
    stepPerMeter = p_stepPerMeter;
    meterPerStep = 1.0/stepPerMeter;
    SetSpeed(speed);
}

void SpeedCommand::SetSpeed(float p_speed){
    speed = p_speed;
    stepPerSampleTime = speed * stepPerMeter * sampleTime;
}

void SpeedCommand::SetSpeedStep(float speed_step){
    stepPerSampleTime = speed_step * sampleTime;
}

float SpeedCommand::GetSpeed(){
    return speed;    
}

void SpeedCommand::Reset(int p_target){
    target = p_target;
    SetSpeed(0);
}
