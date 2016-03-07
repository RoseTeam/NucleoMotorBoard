#include "mbed.h"
#include "SerialCom.h"
#include <iostream>

SerialCom ComPC(PA_11, PA_12);   //handles all the data RX and save them in private variables (coefficient and command ctrl)

Ticker ticker_motor_ctrl;  //handles the motor control loop frequency
Timer timer;

std::string buffer(64,'.');
void tickerInterrupt()
{
    // get current time
    sprintf((char*)buffer.data(), "NUCLEO TIME: %10i", timer.read_ms());
    timer.reset();
}

int main()
{
    ticker_motor_ctrl.attach_us(&tickerInterrupt, 10000);
    timer.start();

    while (1) {

        char * data = ComPC.getMessage();
        if (data)
        {
            ComPC.printf("NUCLEO->PC : %s", data);
            ComPC.releaseMessage();
        }
        //ComPC.printf("NUCLEO->PC : %s", buffer);
        wait_ms(10);
    }
}

