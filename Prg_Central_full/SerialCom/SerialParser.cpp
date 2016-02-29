#include "SerialParser.h"

//#define DEBUG

#ifdef DEBUG
 #define debug_print(...)  SerialParser::printf( ##__VA_ARGS__);
#else
 #define debug_print(...)
#endif

SerialParser::SerialParser(PinName tx, PinName rx , MotorCtrl& _asser, Metrics& _metrics):SerialCom(tx, rx), asser(_asser)
{
    initCst();   // inititate mtotor Ctrl Constants to define values in the .h, waiting for further notice from the COM
}

/*SerialParser::~SerialParser()
{
    //dtor
}*/

void SerialParser::initCst(){
    KpPoAng = KP_POLAR_ANGLE;
    KiPoAng = KI_POLAR_ANGLE;
    KdPoAng = KD_POLAR_ANGLE;
    KiPoAngSat = KI_POLAR_ANGLE_SAT;

    KpPoLin = KP_POLAR_LINEAR;
    KiPoLin = KI_POLAR_LINEAR;
    KdPoLin = KD_POLAR_LINEAR;
    KiPoLinSat = KI_POLAR_LINEAR_SAT;
    
    odoEnabled = true;
    metricsEnabled = false;
}

int getFloat(char* data, float *float_value){
    uint32_t val;
    if (sscanf(data, "%X", &val) == 0) return 0;
     *float_value = *((float*)&val);
    return 1;
}

int getInt(char* data, int *int_value){
    uint32_t val;
    if (sscanf(data, "%x", &val) == 0) return 0;
     *int_value= val;
    return 1;
}


void SerialParser::printMetrics(){
    static int count;
    if (metricsEnabled){   
        SerialParser::printf("%d;%d;%d;%f;%d;%d;%f;\n",
                count, asser.distanceCommand.target, asser.distance, asser.pidDistanceOutput,
                       asser.angleCommand.target, asser.angle, asser.pidAngleOutput                                               
                );

    }
    else count = 0;
}

uint8_t computeCs(char* buff, int len){
    uint8_t cs=0;
    for (int i = 0 ; i < len; i++){
        cs +=  buff[i];    
    }
    return cs;
}

bool SerialParser::verifyCs(char* buff){
    int len = strlen(buff);
    debug_print("len %d ",len);
    if (len < 3) return false;
    uint8_t readCs;
    uint8_t bkpCmd = buff[2];
    buff[2] = 0;
    if (sscanf(buff, "%hhx", &readCs ) == 0) return false;
    buff[2] = bkpCmd;
    uint8_t cs = 0;
    for (int i = 2 ; i < len; i++){
        cs +=  buff[i];    
    }
    debug_print("cs %hhx rd %hhx\n",cs,readCs);
    return (cs == readCs);
       
}

void SerialParser::printOdo(){
    if (odoEnabled){
        char buff[64];
        float x = asser.ODO_X;
        float y = asser.ODO_Y;
        float t = asser.ODO_Theta;
        float s = asser.distanceCommand.GetSpeed();
        float w = asser.angleCommand.GetSpeed();
        int len = sprintf(buff, "O%x;%x;%x;%x;%x\0", *((uint32_t *)&x), *((uint32_t *)&y), *((uint32_t *)&t), *((uint32_t *)&s), *((uint32_t *)&w));
        uint8_t cs = computeCs(buff, len);
        
        char buff2[64];
    int uuid = asser.commandUUID;
    int running = asser.distanceCommand.isRunning() | asser.angleCommand.isRunning() ;
    float blockDist = asser.distanceCommand.getBlockageValue();
    float blockAngle = asser.angleCommand.getBlockageValue();
    int len2 = sprintf(buff2, "S%x;%x;%x;%x\0", uuid, running, *((uint32_t *)&blockDist), *((uint32_t *)&blockAngle));
    uint8_t cs2 = computeCs(buff2, len2);
    //SerialParser::printf("%02X%s\n",cs,buff);        
        
        BufferedSerial::printf("%02X%s\n%02X%s\n",cs,buff,cs2,buff2);
    }
}

void SerialParser::printRobotStatus(){


}

//Function to be called when we have received the full packet
bool SerialParser::interpretData(){
    float a,b,c,d;
    char *data = getMessage();
    if (data == NULL) return false;
    debug_print("%s\n",data);
    if (verifyCs(data) == false){
        debug_print("error cs\n");
        releaseMessage();
        return true;
    }
    float incom_data;
    if (getFloat(data+3, &incom_data) == 0 ){
        goto scanf_error;
    }
    switch(data[2]){

    case 'T':   Ttwist = incom_data; debug_print("Ttwist %f\n", Ttwist); asser.angleCommand.SetSpeed(Ttwist); break;
    case 'V':   Vtwist = incom_data; debug_print("Vtwist %f\n", Vtwist); asser.distanceCommand.SetSpeed(Vtwist); break;
    
    case 'C':   distance = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                angle = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                finalDistanceSpeed = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                finalAngleSpeed = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getInt(data, &uuid) == 0) goto scanf_error;
                asser.setTarget(distance, angle, finalDistanceSpeed, finalAngleSpeed, uuid);
                break;
                
    case 'K':   a = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                b = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                finalDistanceSpeed = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getInt(data, &mode) == 0) goto scanf_error;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getInt(data, &uuid) == 0) goto scanf_error;
                asser.setTargetXY(a, b, finalDistanceSpeed, mode, uuid);
                break;
                
    case 'P':   a = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                b = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                c = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                d = incom_data;
                asser.distanceCommand.setParameters(a,b,c,d);
                break;
                
    case 'Q':   a = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                b = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                c = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                d = incom_data;
                asser.angleCommand.setParameters(a,b,c,d);
                break;
                
    case 'E':   a = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                b = incom_data;
                asser.distanceCommand.setThreshold(a);
                asser.angleCommand.setThreshold(b);
                break;
                
    case 'A':   data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getInt(data, &uuid) == 0) goto scanf_error;
                asser.setTargetAngle(incom_data, uuid);
                break;

    case 'D':   //emergency stop
                asser.distanceCommand.stop((bool)(incom_data));
                asser.angleCommand.stop((bool)(incom_data));
                break;
    
    case 'S':   SStatus = (int)incom_data;
                asser.distanceCommand.Reset(asser.distance);
                asser.distanceCommand.running = false;
                asser.angleCommand.Reset(asser.angle);
                asser.angleCommand.running = false;
                asser.commandUUID = 0;
                break;
    case 'U':   UPower = (bool)incom_data; debug_print("UPower %d\n", UPower); asser.enable(UPower); break;
    case 'M':   debug_print("metrics %d\n", (bool)incom_data); metricsEnabled = (bool)incom_data; break;
    case 'O':   debug_print("odo %d\n", (bool)incom_data); odoEnabled = (bool)incom_data; break;
    


    case '{':   KpPoLin = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                KiPoLin = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                KdPoLin = incom_data;
                debug_print("lin pid %f %f %f\n", KpPoLin, KiPoLin, KdPoLin);
                asser.PidDistance.SetTunings(KpPoLin, KiPoLin, KdPoLin);
                break;

    case '(':   KpPoAng = incom_data; 
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                KiPoAng = incom_data;
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                KdPoAng = incom_data;
                debug_print("ang pid %f %f %f\n", KpPoAng, KiPoAng, KdPoAng);
                asser.PidAngle.SetTunings(KpPoAng, KiPoAng, KdPoAng);
                break;

    case 'x':   asser.setX(incom_data);
                break;

    case 'y':   asser.setY(incom_data);
                break;

    case 'w':   asser.setAngle(incom_data);
                break;
                
    case 'c':   asser.setTickToAngle(incom_data);
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                asser.setTickToDistance(incom_data);
                break;
                                
    case 'd':   asser.angleCommand.SetSampleTime(incom_data);
                data = strchr(data,';'); if (data == NULL) goto scanf_error;
                data += 1;
                if (getFloat(data, &incom_data) == 0) goto scanf_error;
                asser.distanceCommand.SetSampleTime(incom_data);
                break;
                        
                
    default: debug_print("error\n"); break;
    }
    goto end_parse;
            
scanf_error:
    debug_print("error scanf\n");       
end_parse:
    releaseMessage();
    return true;
    
}
