#ifndef ASSERV_H
#define ASSERV_H

#include <stdint.h>
#include "SpeedCommand.h"


class Asserv: public SpeedCommand{
    
    public:
    Asserv();
    
    void setTarget(float finalTarget, float finalSpeed);
    void stop(bool emergency);
    void Compute();
    bool isRunning();
    void setParameters(float acc, float dec, float decEmergency, float maxSpeed);
    
    
    //private
    bool running;
    bool emergencyStop;
    float acc;
    float dec;
    float decEmergency;
    float maxSpeed;
    float finalTarget;
    float finalSpeed;
    int initialSens;
    
    float d_Remaining;
    float d_freinage;


};

#endif