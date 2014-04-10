/* 
 * File:   AtmegaDevice.cpp
 * Author: samuel
 * 
 * Created on 28 January, 2014, 10:00 PM
 */

#include <interface/atmega/AtmegaDevice.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <string>

AtmegaDevice::AtmegaDevice() {
    separator = ' ';
}

AtmegaDevice::AtmegaDevice(const AtmegaDevice& orig) {
}

AtmegaDevice::~AtmegaDevice() {
}

void AtmegaDevice::connect() {
    serial.connect((char *) comm_port.c_str(), baud_rate, spNONE);
    usleep(100);
}

void AtmegaDevice::disconnect() {
    serial.disconnect();
    usleep(100);
}

void AtmegaDevice::recieve(int& data, int size) {
    char character;
    data = 0;

    while (serial.getChar() != separator) {
        usleep(100);
    }

    for (int i = 0; i < size; i++) {
        character = serial.getChar() - '0';
        usleep(100);
        data = data * 10 + character;
    }
}

void AtmegaDevice::send(int data, int size) {
    // Last 'size' digits

    char format_size[2];
    sprintf(format_size, "%d", size);
    char *padded_msg = (char*) calloc(sizeof (char), size + 1);
    std::string format;
    format = std::string("%0") + std::string(format_size) + std::string("d");
    sprintf(padded_msg, format.c_str(), data);
    std::string msg(padded_msg);

    serial.sendChar(separator);
    usleep(100);

    for (int i = 0; i < size; i++) {
        serial.sendChar(msg.at(i));
        usleep(100);
    }
}