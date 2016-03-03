#include "USBSerialCom.h"
#include "MotorCtrl.h"
//#include "MotorDriver.h"

//add the timeout.reset();

//USBSerialCom::USBSerialCom(Serial& _pc, MotorCtrl& _asser) : pc(_pc), asser(_asser){
USBSerialCom::USBSerialCom(void) : 
    pc(USBTX, USBRX),
    asser(0)
{
    // incomming_message_type = 0;     
     //nbr_incom_char = 0;  
    UPower = true;        
    sign = false;
    
  
    
//------------------------------------
// Hyperterminal configuration
// 460800 bauds, 8-bit data, no parity
//------------------------------------
    pc.baud(460800);
    
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

void USBSerialCom::processSerialPort()
{
    
    while(pc.readable()) 
    {           
        char c = pc.getc();            
           //pc.f("%i-",c);
        if (incomming_message_type != 0)   //if it is not the first bit of the packet
        { 
       // pc.printf("D%c!",c);
            if (nbr_incom_char < NBR_CHAR_NAV)
            {                           
                if(c>= 47 &&  c < 58) // if the character received is a number                    
                {
                    inputString[nbr_incom_char] = c;                    
                    nbr_incom_char++;
                           
                }
                else if(c == '!')   //if the character received is end of the packet
                {
                    inputString[nbr_incom_char] = c;                    
                    nbr_incom_char++;
                    interpretData(); 
                    nbr_incom_char = 0; //pc.printf("\n");   
                    sign = false;
                         
                }
                else if (c == 45)
                { 
                    sign = true;
                }
                else                // charachter not recognized, we cancel
                {
                    incomming_message_type = 0;
                    nbr_incom_char = 0;
                    sign = false;                      
                }                                            
            }
            else //default , packet overwhelmed
            {
                pc.printf("Ddef_o!");
                nbr_incom_char = 0;
                incomming_message_type = 0;
                sign = false;
            }
        }
        else //if it is the first bit of the packet, check if it is a standard message
        {
            if (c == 'X' || c == 'Y' || c == 'A' || c == 'L' || c == 'R' || c == 'T' || c == 'V' || c == 'S' || c == 'U' || c =='I')
                { incomming_message_type = c;  }
            else if (c == '{' || c == '}' || c == '^' || c == '=' || c == '(' || c == ')' || c == '_' || c == '|')
                { incomming_message_type = c; }
            else 
                {
                 //Unknown incomming data   
                }
        }
    }
    
}

bool USBSerialCom::checkTimeOut()
{
    //pc.printf("DO%i!",t_timeout_com.read_ms() );
    if (t_timeout_com.read_ms() > COM_TIMEOUT  )
    {                 
           /*Xorder = 0;
           Yorder = 0; 
           Aorder = 0; 
           Lspeed = 0; 
           Rspeed = 0; 
           Ttwist = 0.0; 
           Vtwist = 0.0; 
           SStatus = 0; 
           UPower = 1;      */
       
           return 1;
    }
        
    else { return 0; }
}
 

//Function to be called when we have received the full packet
int USBSerialCom::interpretData()
{    
    int i = 0;
    long incom_data = 0; 
       
    while(inputString[i] != '!')                
    { 
        signed char incom_byte = inputString[i]-48;
            
        if (incom_byte >= 0 && incom_byte < 10)
        { 
			incom_data = incom_data * 10 + incom_byte;
		}
        else
        { 
        //default nbr received
        }                
        i++;
    }

    if(sign) {  incom_data = -incom_data; }
        
	switch (incomming_message_type)
	{
	case 'X': {Xorder = incom_data; t_timeout_com.reset(); /*myled = !myled; pc.printf("%i",Xorder);*/ break; }
	case 'Y': {Yorder = incom_data; t_timeout_com.reset(); break; }
	case 'A': {Aorder = incom_data; t_timeout_com.reset(); break; }
	case 'L': {Lspeed = incom_data; t_timeout_com.reset(); break; }
	case 'R': {Rspeed = incom_data; t_timeout_com.reset(); break; }
	case 'T': {Ttwist = incom_data; t_timeout_com.reset(); break; }  //pc.printf("DT11!",incom_data);}
	case 'V': {Vtwist = incom_data; t_timeout_com.reset(); break; } //pc.printf("DV55!",incom_data);}
	case '{': {KpPL = incom_data; t_timeout_com.reset(); break; }
	case '}': {KdPL = incom_data; t_timeout_com.reset(); break; }
	case '^': {KiPL = incom_data; t_timeout_com.reset(); break; }
	case '=': {KiPLS = incom_data; t_timeout_com.reset();  break; }
	case '(': {KpPA = incom_data; t_timeout_com.reset(); break; }
	case ')': {KdPA = incom_data; t_timeout_com.reset(); break; }
	case '_': {KiPA = incom_data; t_timeout_com.reset(); break; }
	case '|': {KiPAS = incom_data; t_timeout_com.reset(); break; }

	case 'S': {SStatus = true; t_timeout_com.reset(); break; }
	case 'U': {UPower = (bool)incom_data;  pc.printf("DO%i!", incom_data); t_timeout_com.reset(); break; }
	case 'I': {sendCoeffs(); break; }
	}
        
    //pc.printf("D:dr!");  
         
    incomming_message_type = 0;
    nbr_incom_char = 0;
    sign = false;
           
    return 1;
}

void USBSerialCom::setSStatus(bool value){
    SStatus = value;    
}

    
void USBSerialCom::sendCoeffs(){
    static int count = 0;
    count ++;
    if (count > 20) {
        pc.printf("D-----!");
        pc.printf("DKpPA%lf!", getKpPA());
        pc.printf("DKdPA%lf!", getKdPA());
        pc.printf("DKiPA%lf!", getKiPA());
        pc.printf("DKpPL%lf!", getKpPL());
        pc.printf("DKdPL%lf!", getKdPL());
        pc.printf("DKiPL%lf!", getKiPL());
        pc.printf("D----!");
        count = 0;
    }
}

void USBSerialCom::sendFeedback(long pidL, long pidR, float pidA, float pidT){
    static int count82 = 0;
    count82 ++;
    if (count82 > 20) 
    {
        pc.printf("DL%ld!", long(pidL));
        pc.printf("DR%ld!", long(pidR));
        pc.printf("DO%f!", pidA);
        pc.printf("DD%f!", pidT);
         //   pc.printf("DU%i!", getUPower());
        count82 = 0;
    }
}
    
//for debug purposes
void USBSerialCom::sendHeartBeat(long data){
    pc.printf("DL%ld!",long(data));
}
 
void USBSerialCom::printMetrics(){
	/*
	static int count;
    if (metricsEnabled){   
        SerialParser::printf("%d;%d;%d;%f;%d;%d;%f;\n",
                count, asser.distanceCommand.target, asser.distance, asser.pidDistanceOutput,
                
                       asser.angleCommand.target, asser.angle, asser.pidAngleOutput                                               
                );

    }
    else count = 0;
	*/
}



void USBSerialCom::printOdo()
{   
    if (asser) 
    {        //pc.printf("Yr");
        pc.printf("X%ld!",long(asser->getODO_X()*100));
        pc.printf("Y%ld!",long(asser->getODO_Y()*100));
        pc.printf("A%ld!",long(asser->getODO_Theta()*100));
            
        pc.printf("L%ld!",(int)Vtwist);
        pc.printf("R%ld!",(int)Ttwist); 
    }                
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
    
    else pc.printf("DNoAsserOdo!");
}

void USBSerialCom::printRobotStatus(){

    pc.printf("Dok!"); 
}

