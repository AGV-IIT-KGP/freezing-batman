/* 
 * File:   AtmegaDevice.hpp
 * Author: samuel
 *
 * Created on 28 January, 2014, 10:00 PM
 */

#ifndef ATMEGADEVICE_HPP
#define	ATMEGADEVICE_HPP

#include <string>
#include <interface/atmega/Serial.hpp>

class AtmegaDevice {
public:
    AtmegaDevice();
    AtmegaDevice(const AtmegaDevice& orig);
    virtual ~AtmegaDevice();

    void SetBaud_rate(int baud_rate) {
        this->baud_rate = baud_rate;
    }

    void SetComm_port(std::string comm_port) {
        this->comm_port = comm_port;
    }
    
    void connect();
    void disconnect();
    void recieve(int& data, int size);
    void send(int data, int size);

private:
    char separator;
    int baud_rate;
    std::string comm_port;
    Tserial serial;
};

#endif	/* ATMEGADEVICE_HPP */

