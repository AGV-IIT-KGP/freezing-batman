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
    public:
        virtual bool connect() = 0;
        virtual bool fetch() = 0;
        virtual bool disconnect() = 0;
        virtual void publish(int frame_id) = 0;
    };
}
#endif	/* SENSOR_HPP */

