/* 
 * File:   Sensor.cpp
 * Author: Satya Prakash
 * 
 * Created on December 13, 2013, 6:32 PM
 */

#include "environment/Sensor.hpp"
#include "ros/ros.h"

namespace environment {

    Sensor::Sensor(int argc, char **argv) {
        ros::init(argc, argv, std::string("Blah"));
    }

    Sensor::~Sensor() {
    }

}
