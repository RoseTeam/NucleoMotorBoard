#include "mbed.h"
#include "USBSerialCom.h"

#include "MotorCtrl.h"
//#include "MotorDriver.h"


#define MOTOR_CTRL_TICKER_PERIOD 4000
#define TICKER_MAX_DURATION 2000

DigitalOut myled(LED1);

//MotorDriver motors(MOTOR_1VIT,MOTOR_1DIR,MOTOR_2VIT,MOTOR_2DIR);
//Metrics metrics;
//SerialParser ComPC(SERIAL_TX, SERIAL_RX, asser, metrics);   //handles all the data RX and save them in private variables (coefficient and command ctrl)
//SerialParser ComPC(PA_11, PA_12, asser, metrics);   //handles all the data RX and save them in private variables (coefficient and command ctrl)

//------------------------------------
// Hyperterminal configuration
// 500000 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX);   //create a Serial COM port over the mini USB plug of the ST Nucleo

//MotorCtrl asser

USBSerialCom ComPC(pc);   //handles all the data RX and save them in private variables (coefficient, command setpoint and odom)
//USBSerialCom ComPC(pc,asser); 

Ticker ticker_motor_ctrl;  //handles the motor control loop frequency


MotorCtrl asser(ComPC);// handles the motor control algorithm (access to ComPc to receive the setpoint commands and send debugs)


Timer t_com;



static int taskSelector = 0; // odometry 1/4   asserv 4/4
int nbReceivedData = 0;
void tickerInterrupt()
{
    asser.Interrupt_Handler_Encoder();
}

int main()
{
    ticker_motor_ctrl.attach_us(&tickerInterrupt, MOTOR_CTRL_TICKER_PERIOD);
    
    pc.attach(&ComPC, &USBSerialCom::serialCallback);

    t_com.start();

    //TODO: Enable a WatchDog !!!

    int count = 0;
    while (1) {
        //asser.UpdateCmd();
        if (asser.DataAvailable()) {  
            asser.ComputeOdometry();          
            asser.Compute();         
            
            }
          
        
    
        
    if (t_com.read_ms() > 50)
      {
        pc.printf("X%ld!",long(asser.getODO_X()*100));
        pc.printf("Y%ld!",long(asser.getODO_Y()*100));
        pc.printf("A%ld!",long(asser.getODO_Theta()*100));
        
        pc.printf("L%ld!",asser.getWheelL());
        pc.printf("R%ld!",asser.getWheelR()); 
        
        if( ComPC.checkTimeOut() ) {   pc.printf("D:TOut!");   }
        
        t_com.reset();
        
        //ComPC.sendCoeffs();
      }
      
      
                
        //ComPC.interpretData();
    }
}
