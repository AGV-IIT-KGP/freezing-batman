#include <iostream>
#include "../SerialPortLinux/serial_lnx.h"

using namespace std;

namespace Utils
{
  class Communication
  {
    private:
      Tserial *port;

    public:
      Communication(char *commPort, int baudRate, bool verbose)
      {
        this.port = new Tserial();
        this.port->connect(commPort, baudRate, spNone, verbose);
      }
  }

  class Device : public Communication
  {

  }
  
  class IMU : public Device
  {
    private:
      double yaw;
      static char *commPort = "/dev/ttyUSB0";
      static int baudRate = 19200;

    public:
      
  }

  class Sonar : public Device
  {
    
  }
}
