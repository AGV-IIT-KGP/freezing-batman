/* 
 * File:   Serial.hpp
 * Author: samuel
 *
 * Created on 28 January, 2014, 7:02 PM
 */

#ifndef SERIAL_HPP
#define	SERIAL_HPP

#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

enum serial_parity {
    spNONE, spODD, spEVEN
};

class Tserial {
private:
    int fd_serialport;
    //char              port[10];                      // port name "com1",...//dev//ttyS0
    //int               rate;                          // baudrate
    serial_parity parityMode;
    bool v;

public:
    Tserial();
    ~Tserial();
    int connect();
    int connect(char * device);
    int connect(char *device, int rate_arg, serial_parity parity_arg, bool verbose = false);
    void disconnect(void);

    int sendArray(char *buffer, int len);
    int getArray(char *buffer, int len);

    int bytesToRead();
    void clear();

    void sendChar(char c);
    char getChar(void);

    /*
    protected:
        char              port[10];                      // port name "com1",...
        int               rate;                          // baudrate
        serial_parity     parityMode;
        HANDLE            serial_handle;                 // ...

        // ++++++++++++++++++++++++++++++++++++++++++++++
        // .................. EXTERNAL VIEW .............
        // ++++++++++++++++++++++++++++++++++++++++++++++
    public:
                      Tserial();
                     ~Tserial();
        int           connect          (char *port_arg, int rate_arg,
                                        serial_parity parity_arg);
        void          sendChar         (char c);
        void          sendArray        (char *buffer, int len);
        char          getChar          (void);
        int           getArray         (char *buffer, int len);
        int           getNbrOfBytes    (void);
        void          disconnect       (void);
     */
};

#endif	/* SERIAL_HPP */

