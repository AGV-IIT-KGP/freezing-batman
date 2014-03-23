#include <stdlib.h>
#include <stdio.h>
#include <../include/SerialPortLinux/serial_lnx.h>
#include <stdexcept>
//#include "../../Modules/devices.h"

#define UART_COMM_PORT "/dev/ttyUSB0"
#define UART_BAUD_RATE 19200

Tserial *p;

void Init()
{
  p = new Tserial();
  p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
  usleep(100);

  //p->sendChar('w');
  //usleep(100);
}

void Turn(int l, int r)
{
  p->sendChar('w');
  usleep(100);
  
  p->sendChar('0'+ l/10);
  usleep(100);
  p->sendChar('0'+ l%10);
  usleep(100);
  p->sendChar('0'+ r/10);
  usleep(100);
  p->sendChar('0'+ r%10);
  usleep(100);
}

void Terminate()
{
  p->sendChar(' ');
  usleep(100);
  
  p->disconnect();
  usleep(100);
}

int main()
{
  int i;
  Init();
while(1)
{
  Turn(10, 10);
  //usleep(100*1000);
}
  p->disconnect();
  usleep(5000 * 1000);
  
  p->connect(UART_COMM_PORT, UART_BAUD_RATE, spNONE);
  usleep(100);

  Terminate();
  
  return 0;
}
