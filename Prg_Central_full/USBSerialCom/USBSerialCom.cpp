#include "USBSerialCom.h"
#include "MotorCtrl.h"
#include "mbed.h"


//add the timeout.reset();

//USBSerialCom::USBSerialCom(Serial& _pc, MotorCtrl& _asser) : pc(_pc), asser(_asser){
USBSerialCom::USBSerialCom(Serial& _pc) : pc(_pc){
    // incomming_message_type = 0;     
     //nbr_incom_char = 0;  
    UPower = true;        
    sign = false;
    pc.baud(115200);
    
    KpPL = KP_POLAR_LINEAR;
    KdPL = KD_POLAR_LINEAR;
    KiPL = KI_POLAR_LINEAR;
    KiPLS = KI_POLAR_LINEAR_SAT;
    
    KpPA = KP_POLAR_ANGLE;
    KdPA = KD_POLAR_ANGLE;
    KiPA = KI_POLAR_ANGLE;
    KiPAS = KI_POLAR_ANGLE_SAT;
    
    t_timeout_com.start();
            
}


bool USBSerialCom::checkTimeOut()
{
    if (t_timeout_com.read_ms() > COM_TIMEOUT  )
        {
         
       Xorder = 0;
       Yorder = 0; 
       Aorder = 0; 
       Lspeed = 0; 
       Rspeed = 0; 
       Ttwist = 0.0; 
       Vtwist = 0.0; 
       SStatus = 0; 
       UPower = 1;        
       
       return 1;
                }
    else { return 0;}
}
        
void USBSerialCom::serialCallback()
{          
  while(pc.readable()) 
    {   
        
        char c = pc.getc();            
           //pc.printf("%i-",c);
        if (incomming_message_type != 0)   //if it is not the first bit of the packet
        { 
       // pc.printf("D%c!",c);
            if (nbr_incom_char < NBR_CHAR_NAV)
                {                           
                if(c>= 47 &&  c < 58) // if the character received is a number                    
                    {
                        inputString[nbr_incom_char] = c;                    
                        nbr_incom_char++;
                        //pc.printf("D:!");
                        //pc.printf("g");  
                    }
                else if(c == '!')   //if the character received is end of the packet
                    {
                        inputString[nbr_incom_char] = c;                    
                        nbr_incom_char++;
                        interpretData(); 
                        nbr_incom_char = 0; //pc.printf("\n");   
                        sign = false;
                         //pc.printf("D[!");
                    }
                else if (c == 45)
                   { sign = true;}
                else                // charachter not recognized, we cancel
                    {
                        incomming_message_type = 0;
                        nbr_incom_char = 0;
                        sign = false;
                         
                        
                        //pc.printf("def");  
                    }                                            
                }
            else
                {//default , packet overwhelmed
                pc.printf("Ddef_o!");
                nbr_incom_char = 0;
                incomming_message_type = 0;
                sign = false;
                }
        }
        else //if it is the first bit of the packet, check if it is a standard message
        {
            if (c == 'X' || c == 'Y' || c == 'A' || c == 'L' || c == 'R' || c == 'T' || c == 'V' || c == 'S' || c == 'U' )
                { incomming_message_type = c; /*pc.printf("u");*/ }
            else if (c == '{' || c == '}' || c == '^' || c == '=' || c == '(' || c == ')' || c == '_' || c == '|')
                { incomming_message_type = c; /*pc.printf("u");*/ }
            else 
                {
                 //Unknown incomming data   
                }
        }
    } 
   
           
 }

//Function to be called when we have received the full packet
int USBSerialCom::interpretData(){    
        int i = 0;
        long incom_data = 0; 

       
        while(inputString[i] != '!')                
            { 
            char incom_byte = inputString[i]-48;
            
            if (incom_byte >= 0 &&  incom_byte < 10)
                { incom_data = incom_data*10 + incom_byte;}
            else
                { 
                //default nbr received
                }                         
            i++;
            }
        if(sign) {  incom_data = -incom_data; }
        
        if (incomming_message_type == 'X') {Xorder = incom_data; t_timeout_com.reset(); /*myled = !myled; pc.printf("%i",Xorder);*/}
        else if (incomming_message_type == 'Y') {Yorder = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == 'A') {Aorder = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == 'L') {Lspeed = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == 'R') {Rspeed = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == 'T') {Ttwist = incom_data; t_timeout_com.reset();}//pc.printf("DT11!",incom_data);}
        else if (incomming_message_type == 'V') {Vtwist = incom_data; t_timeout_com.reset();} //pc.printf("DV55!",incom_data);}
        else if (incomming_message_type == 'S') {SStatus = true; t_timeout_com.reset();}
        else if (incomming_message_type == 'U') {UPower = (bool)incom_data;  pc.printf("DO%i!",incom_data); t_timeout_com.reset();}
       
        else if (incomming_message_type == '{') {KpPL = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == '}') {KdPL = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == '^') {KiPL = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == '=') {KiPLS = incom_data; t_timeout_com.reset(); }
        else if (incomming_message_type == '(') {KpPA = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == ')') {KdPA = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == '_') {KiPA = incom_data; t_timeout_com.reset();}
        else if (incomming_message_type == '|') {KiPAS = incom_data; t_timeout_com.reset();}
        
        
        //pc.printf("D:dr!");  
         
        incomming_message_type = 0;
        nbr_incom_char = 0;
        sign = false;
           
    return 1;
}

int USBSerialCom::getTtwist(){
    return Ttwist;
}

int USBSerialCom::getVtwist(){
    return Vtwist;
}

bool USBSerialCom::getSStatus(){
    return SStatus;
}

bool USBSerialCom::getUPower(){
    return UPower;
}


float USBSerialCom::getKpPL() {
    return KpPL/1000.0;
}

float USBSerialCom::getKdPL(){
    return KdPL/1000.0;
}

float USBSerialCom::getKiPL(){
    return KiPL/1000.0;
}

float USBSerialCom::getKiPLS(){
    return KiPLS/1000.0;
}

    
float USBSerialCom::getKpPA(){
    return KpPA/1000.0;
}

float USBSerialCom::getKdPA(){
    return KdPA/1000.0;
}

float USBSerialCom::getKiPA(){
    return KiPA/1000.0;
}

float USBSerialCom::getKiPAS(){
    return KiPAS/1000.0;
}


void USBSerialCom::setSStatus(bool value){
    SStatus = value;    
}

    
void USBSerialCom::sendCoeffs(){
    static int count = 0;
    count ++;
    if (count > 20) {
        pc.printf("D-----!");
        pc.printf("DKpPA%lf!",getKpPA());
        pc.printf("DKdPA%lf!",getKdPA());
        pc.printf("DKiPA%lf!", getKiPA());
        pc.printf("DKpPL%lf!", getKpPL());
        pc.printf("DKdPL%lf!", getKdPL());
        pc.printf("DKiPL%lf!", getKiPL());
        pc.printf("D----!");
        count = 0;
        }
}

void USBSerialCom::sendFeedback(long pidL, long pidR, float pidA, float pidT){
    static int count = 0;
    count ++;
    if (count > 100) {
    pc.printf("DL%ld!",long(Ttwist));
        pc.printf("DR%ld!",long(Vtwist));
        //pc.printf("DA%lf!", pidA);
        //pc.printf("DT%lf!", pidT);
        pc.printf("DU%i!", getUPower());
        count = 0;
        }
}
    
//for debug purposes
void USBSerialCom::sendHeartBeat(long data){
    pc.printf("DL%ld!",long(data));
}
 
void USBSerialCom::printMetrics(){
    static int count;
    if (metricsEnabled){   
       /* SerialParser::printf("%d;%d;%d;%f;%d;%d;%f;\n",
                count, asser.distanceCommand.target, asser.distance, asser.pidDistanceOutput,
                
                       asser.angleCommand.target, asser.angle, asser.pidAngleOutput                                               
                );*/

    }
    else count = 0;
}


void USBSerialCom::printOdo(){
    /*if (odoEnabled){
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
    }*/
}

void USBSerialCom::printRobotStatus(){

 pc.printf("Dok!");
 
}

