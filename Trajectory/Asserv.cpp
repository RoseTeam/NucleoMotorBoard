#include "Asserv.h"
#define abs(x) ((x) < 0 ? - (x) : (x))

Asserv::Asserv()
{
    running = false;
    acc = 0.10;
    dec = 0.10;
    decEmergency = dec * 2.0;
    maxSpeed = 0.15;
}

void Asserv::setParameters(float pacc, float pdec, float pdecEmergency, float pmaxSpeed)
{
    acc = abs(pacc);
    dec = abs(pdec);
    decEmergency = abs(pdecEmergency);
    maxSpeed = abs(pmaxSpeed);
}

void Asserv::setTarget(float pfinalTarget, float pfinalSpeed)
{
    if (pfinalTarget == 0.0){
        running = false;
        emergencyStop = false;
        return;
    } 
    finalTarget = pfinalTarget + (target * meterPerStep);
    finalSpeed = pfinalSpeed;
    running = true;
    emergencyStop = false;
    if (pfinalTarget > 0) {
        initialSens = 1;
    } else {
        initialSens = -1;
    }
}

void Asserv::stop(bool emergency)
{
    float actualSpeed = GetSpeed();
    /* distance theorique (temps continu) pour annuler la vitesse avec un freinage maximum et constant*/
    if (emergency){
        d_freinage  = (actualSpeed*actualSpeed) * .5 / decEmergency;
    } else {
        d_freinage  = (actualSpeed*actualSpeed) * .5 / dec;    
    }
    setTarget(initialSens * d_freinage, 0.0);
    emergencyStop = emergency;

}

void Asserv::Compute()
{
    if (running) {
        float actualSpeed = GetSpeed();
        float newSpeed = actualSpeed;
        d_Remaining = finalTarget - (target * meterPerStep);
        int sens = 1;
        if (d_Remaining < 0) sens = -1;
        if (initialSens != sens) { // target reachead
            running = false;
            if (finalSpeed == 0.0) SetSpeed(finalSpeed);
            return;
        }
        /* distance theorique (temps continu) pour annuler la vitesse avec un freinage maximum et constant*/
        d_freinage  = (actualSpeed*actualSpeed - finalSpeed*finalSpeed) * .5 / dec;

        if (abs(d_Remaining) > d_freinage ) {
            if (abs(actualSpeed) < maxSpeed ) {
                newSpeed += (sens * acc * sampleTime);
            }
        } else {
            if (emergencyStop) {
                newSpeed -= (sens * decEmergency * sampleTime);
            } else {
                newSpeed -= (sens * dec * sampleTime);
            }
        }
        SetSpeed(newSpeed);
    }
    ComputeSpeed();
}

bool Asserv::isRunning()
{
    return running;
}