/* 
 * File:   Sensor.hpp
 * Author: Satya Prakash
 *
 * Created on December 12, 2013, 11:04 PM
 */

#ifndef SENSOR_HPP
#define	SENSOR_HPP

namespace environment {

    class Sensor {
        virtual bool connect();
        virtual bool fetch();
        virtual bool disconnect();
    public:
        virtual void getData();
    };
}
#endif	/* SENSOR_HPP */

