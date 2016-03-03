#ifndef SPEEDCOMMAND_H
#define SPEEDCOMMAND_H

#include <stdint.h>


class SpeedCommand
{
    public:
        SpeedCommand();

        void SetSpeed(float p_speed); // m.s-1
        void SetSpeedStep(float speed_step); // step.s-1
        float GetSpeed();     
        void SetSampleTime(float p_samplePeriod); // sample period in seconds
        void SetStepPerMeter(float p_stepPerMeter);
        void ComputeSpeed();
        void Reset(int p_target);
        void blockageDetector(float actualPosition);
        float getBlockageValue();
        void setThreshold(float threshold);
        
        int target;

    //private:
        float speed;
        float stepPerMeter;
        float meterPerStep;
        float stepPerSampleTime;
        float sampleTime;
        float remainingStep;
        
        float blockageTreshold;
        float blockageValue;
};

#endif // SPEEDCOMMAND_H
