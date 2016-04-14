#include "mbed.h"
#include "SerialParser.h"
#include "MotorCtrl.h"
#include "MotorDriver.h"

#define MAX(a,b) (((a)>(b))?(a):(b))

#define TICKER_PERIOD 4000
#define TICKER_MAX_DURATION 2000
DigitalOut myled(LED1);



MotorCtrl asser;  // handles the motor control algorithm (access to ComPc to know the orders)
//MotorDriver motors(MOTOR_1VIT,MOTOR_1DIR,MOTOR_2VIT,MOTOR_2DIR);
Metrics metrics;
//------------------------------------
// Hyperterminal configuration
// 230400 bauds, 8-bit data, no parity
//------------------------------------

//SerialParser ComPC(SERIAL_TX, SERIAL_RX, asser, metrics);   //handles all the data RX and save them in private variables (coefficient and command ctrl)
SerialParser ComPC(PA_11, PA_12, asser, metrics);   //handles all the data RX and save them in private variables (coefficient and command ctrl)

Ticker ticker_motor_ctrl;  //handles the motor control loop frequency



Timer t_com;
Timer t_debug;
Timer t_perf;
int t_perf1, t_perf2, t_perf3, t_perf4;


static int taskSelector = 0; // odometry 1/4   asserv 4/4
int nbReceivedData = 0;
void tickerInterrupt()
{
    asser.Interrupt_Handler();
}

int main()
{
    ticker_motor_ctrl.attach_us(&tickerInterrupt, TICKER_PERIOD);

    //t_com.start();
    t_debug.start();
    t_perf.start();
    //TODO: Enable a WatchDog !!!

    int count = 0;
    while (1) {

        // Check if there is new encoder data
        if (asser.DataAvailable()) {
            //
            asser.Compute();
             ++taskSelector;
            if ((taskSelector & 0x3) == 0x2) { // if xxxxxxx10
                asser.ComputeOdometry();
            }
            if ((taskSelector & 0x1f) == 0x1f) { // if xxxx11111 (xxxxxxx11) // 1 tick after odom
                ComPC.printOdo(); // print data like odometri or metrics
                ComPC.printRobotStatus();
                ComPC.printMetrics();
            }
        }
        ComPC.interpretData();
    }
}
