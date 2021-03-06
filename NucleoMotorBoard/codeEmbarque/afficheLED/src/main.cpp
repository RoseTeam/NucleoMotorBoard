#include "mbed.h"
//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

Serial pc(SERIAL_TX, SERIAL_RX);
//USBSerial pc(USBTX, USBRX);

DigitalOut myled(LED1);

int main() {
	int i = 1;
	pc.printf("Hello World !\n");
	while (1) {
		wait_ms(1300);
		pc.printf("Running since %d seconds.\n", i++);
		myled = !myled;
	}
}