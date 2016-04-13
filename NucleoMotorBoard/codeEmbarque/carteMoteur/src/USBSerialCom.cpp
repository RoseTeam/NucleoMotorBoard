#include "USBSerialCom.h"
#include "MotorCtrl.h"
//#include "MotorDriver.h"

//add the timeout.reset();

//USBSerialCom::USBSerialCom(Serial& _pc, MotorCtrl& _asser) : pc(_pc), asser(_asser){
USBSerialCom::USBSerialCom(void) : 
    pc(USBTX, USBRX),
    asser(NULL)
{
    incoming_message_type = 0;
    nbr_incom_char = 0;
	UPower = true;
    sign = false;
    
  
    
//------------------------------------
// Hyperterminal configuration
// 460800 bauds, 8-bit data, no parity
//------------------------------------
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

void USBSerialCom::processSerialPort()
{   
	while(pc.readable())
    {           
        const char c = pc.getc();            
           //pc.f("%i-",c);

        if (incoming_message_type != 0)   //if it is not the first bit of the packet
        { 
       // pc.printf("D%c!",c);
            if (nbr_incom_char < NBR_CHAR_NAV)
            {                           
                if(c >= 47 &&  c < 58) // if the character received is a number                    
                {
                    inputString[nbr_incom_char] = c;                    
                    nbr_incom_char++;
                }
                else if(c == '!')   //if the character received is end of the packet
                {
                    inputString[nbr_incom_char] = c;                    
                    nbr_incom_char++;
                    if(interpretData()){
						// one valid message has been received
						t_timeout_com.reset();
						pc.printf("Dmsg recu_%s!", inputString);
					}
					incoming_message_type = 0;
                }
                else if (c == 45)
                { 
                    sign = true;
                }
                else // character not recognized, we cancel
                {
                    incoming_message_type = 0;
                }                                            
            }
            else //default, packet overwhelmed
            {
                pc.printf("Ddef_o!");
                incoming_message_type = 0;
            }
        }
        else //if it is the first bit of the packet, check if it is a standard message
        {
			switch (c)
			{
				case 'X':
				case 'Y':
				case 'A':
				case 'L':
				case 'R':
				case 'T':
				case 'V':
				case 'S':
				case 'U':
				case 'I':
				case '{':
				case '}':
				case '^':
				case '=':
				case '(':
				case ')':
				case '_':
				case '|':
				{ 
					incoming_message_type = c;

					// reset message data at beginning of message
					nbr_incom_char = 0;
					sign = false;

					break;
				}
				default:
					//Unknown incomming data   
					break;
			}
        }
    }
    
}

bool USBSerialCom::checkTimeOut()
{
    //pc.printf("DO%i!",t_timeout_com.read_ms() );
    if (t_timeout_com.read_ms() > COM_TIMEOUT)
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
    else { 
		return 0; 
	}
}
 

//Function to be called when we have received the full packet
int USBSerialCom::interpretData()
{    
    long incom_data = 0; 
       
	for (int i = 0; inputString[i] != '!'; i++)
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
    }

    if(sign) { incom_data = -incom_data; }
        
	switch (incoming_message_type)
	{
	case 'X': {Xorder = incom_data; /*myled = !myled; pc.printf("%i",Xorder);*/ break; }
	case 'Y': {Yorder = incom_data; break; }
	case 'A': {Aorder = incom_data; break; }
	case 'L': {Lspeed = incom_data; break; }
	case 'R': {Rspeed = incom_data; break; }
	case 'T': {Ttwist = incom_data; break; }  //pc.printf("DT11!",incom_data);}
	case 'V': {Vtwist = incom_data; break; } //pc.printf("DV55!",incom_data);}
	case '{': {KpPL = incom_data; break; }
	case '}': {KdPL = incom_data; break; }
	case '^': {KiPL = incom_data; break; }
	case '=': {KiPLS = incom_data;  break; }
	case '(': {KpPA = incom_data; break; }
	case ')': {KdPA = incom_data; break; }
	case '_': {KiPA = incom_data; break; }
	case '|': {KiPAS = incom_data; break; }

	case 'S': {SStatus = true; break; }
	case 'U': {UPower = (bool)incom_data;  pc.printf("DU%i!", incom_data); break; }
	case 'I': {sendCoeffs(); break; }
	default:
		pc.printf("D:unknown message type %s!", incoming_message_type);           
		return 0;
	}
        
    //pc.printf("D:dr!");           
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
    {   
		//pc.printf("Yr");
		//pc.printf("D:printodo!");

        pc.printf("X%ld!",long(asser->getODO_X()*100));
        pc.printf("Y%ld!",long(asser->getODO_Y()*100));
        pc.printf("A%ld!",long(asser->getODO_Theta()*100));
            
        pc.printf("L%ld!", Vtwist);
        pc.printf("R%ld!", Ttwist); 
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

