#include "MotorCtrl.h"
#include "SerialParser.h"
#include "mbed.h"

#define PI2 6.283185307179586
#define PI 3.141592653589793

MotorCtrl::MotorCtrl() :
    wheelL(Encoder1_A, Encoder1_B, NC, ENCODER_RES),
    wheelR(Encoder2_A, Encoder2_B, NC, ENCODER_RES),
    PidAngle(&angle, &pidAngleOutput, &angleCommand.target,KP_POLAR_ANGLE, KI_POLAR_ANGLE, KD_POLAR_ANGLE,-1),
    PidDistance(&distance, &pidDistanceOutput, &distanceCommand.target,KP_POLAR_LINEAR, KI_POLAR_LINEAR, KD_POLAR_LINEAR,-1),
    angle_buf(4),
    distance_buf(4)

{
    ODO_X = 0;
    ODO_Y = 0;
    ODO_Theta = 0;
    ODO_Theta_Offset = 0.0;
    ODO_khi = 0.0;
    ODO_ds= 0.0;
    tickToDistance = (PI*R_WHEEL/ENCODER_RES)*54.4/100;
    
    tickToAngle =  PI*R_WHEEL/(WHEEL_B * ENCODER_RES) * PI2 /5.569498538970947;
    
    distanceCommand.SetStepPerMeter(1.0/tickToDistance);
    angleCommand.SetStepPerMeter(1.0/tickToAngle);
    
    isEnabled = 0;
    commandUUID = 0;
    // variable de position initialisées

    mode_deplacement = 1;// sert à déffinir le mode de déplacement : polaire, linèaire...(ici 1 seul mode)
    
}



void MotorCtrl::ComputeOdometry()
{
    deltaDistance = distance - previousDistance;
    previousDistance = distance;
    double trueDeltaDistance = deltaDistance * tickToDistance;
    
    //ODO_ds = deltaDistance/ENCODER_RES;// angle rotation en radians!!!
    
    ODO_Theta = (tickToAngle * angle) + ODO_Theta_Offset; //  mesure la rotation en radian
    
    ODO_DELTA_X = trueDeltaDistance  * cos(ODO_Theta);
    ODO_DELTA_Y = trueDeltaDistance* sin(ODO_Theta);

    ODO_X = ODO_X + ODO_DELTA_X;
    ODO_Y = ODO_Y + ODO_DELTA_Y;
}

void MotorCtrl::setX(float x){
    ODO_X = x;
}

void MotorCtrl::setY(float y){
    ODO_Y = y;
}

void MotorCtrl::setAngle(float Theta){
    ODO_Theta_Offset = Theta - (tickToAngle * angle);
    ODO_Theta = Theta;
}

void MotorCtrl::setTarget(float distance, float angle, float finalDistanceSpeed, float finalAngleSpeed, int uuid){
    commandUUID = uuid;
    distanceCommand.setTarget(distance,finalDistanceSpeed);
    angleCommand.setTarget(angle,finalAngleSpeed);
}

void MotorCtrl::setTargetAngle(float angleAbs, int uuid){
    commandUUID = uuid;
    double angle =  angleAbs - ODO_Theta;
    
    if (angle  > 0.0){
        angle = fmod(angle + PI, 2.0 * PI) - PI;
    } else {
        angle = fmod(angle - PI, 2.0 * PI) + PI;
    }
    angleCommand.setTarget(angle, 0);
}

void MotorCtrl::setTargetXY(float x, float y, float finalDistanceSpeed, int mode, int uuid){ //mode rotation = 1, distance = 2 , both = 3
    commandUUID = uuid;
    x -= ODO_X;
    y -= ODO_Y;
    
    float dist = 0.0;
    float angle = 0.0;
    
    if (x != 0.0 || y != 0.0){
        dist = sqrt(x * x + y * y);
        angle = atan2(y,x) - ODO_Theta;

        if (angle  > 0.0){
            angle = fmod(angle + PI, 2.0 * PI) - PI;
        } else {
            angle = fmod(angle - PI, 2.0 * PI) + PI;
        }
    }
    if (mode & 0x2){
        distanceCommand.setTarget(dist, finalDistanceSpeed);
    }
    if (mode & 0x1){
        angleCommand.setTarget(angle,0);
    }
}

void MotorCtrl::setTickToAngle(float pTickToAngle){
    tickToAngle= pTickToAngle;
    angleCommand.SetStepPerMeter(1.0/pTickToAngle);
} 

void MotorCtrl::setTickToDistance(float pTickToDistance){
    tickToDistance = pTickToDistance;
    distanceCommand.SetStepPerMeter(1.0/pTickToDistance);
}

void MotorCtrl::enable(int is_enabled){
    isEnabled = is_enabled;
    
    distanceCommand.Reset(distance); // reset speed trajectory generator
    angleCommand.Reset(angle);
    
}

void MotorCtrl::SystemCtrl(){    
    
    PidDistance.Compute();
    PidAngle.Compute();
    
    
    float motorL = pidDistanceOutput + pidAngleOutput;
    float motorR = pidDistanceOutput - pidAngleOutput;
    
    if(isEnabled){
        Motors.Motor1(motorL);
        Motors.Motor2(motorR);
    }
    else {
        Motors.Motor1(0);
        Motors.Motor2(0);
    }
}

void MotorCtrl::Interrupt_Handler()
{
    wheelLTick = wheelL.getPulses();
    wheelRTick = wheelR.getPulses();
    
    distance_buf.put((wheelLTick + wheelRTick ));
    angle_buf.put((wheelRTick - wheelLTick));
}

bool MotorCtrl::DataAvailable(){
    if (distance_buf.available()){
        distance = distance_buf;
        angle = angle_buf;
        return true;
    }
    return false;
}

void MotorCtrl::Compute()
{
    // compute target
//    distanceCommand.Compute();
//    angleCommand.Compute();
//    distanceCommand.blockageDetector(distance);
//    angleCommand.blockageDetector(angle);
//    SystemCtrl();

    // TODO : Call Lucas Skywalker !

}

double MotorCtrl::getODO_X(){
    return ODO_X;
}

double MotorCtrl::getODO_Y(){
    return ODO_Y;
}

double MotorCtrl::getODO_Theta(){
    return ODO_Theta;
}

double MotorCtrl::getODO_SPEED_X(){
    return ODO_DELTA_X*TE;
}

double MotorCtrl::getODO_SPEED_Y(){
    return ODO_DELTA_Y*TE;
}

double MotorCtrl::getODO_SPEED_Theta()
{
    return ODO_SPEED_Theta;
}

long MotorCtrl::getWheelL()
{
    return wheelLTick;
}

long MotorCtrl::getWheelR()
{
    return wheelRTick;
}

void MotorCtrl::Debug(){
    
    //ComPC.sendFeedback(ODO_X,ODO_X,pidA,pidT);

}
