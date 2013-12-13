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
        virtual bool connect();
        virtual bool fetch();
        virtual bool disconnect();
        virtual void publish(int frame_id);
    };
}
#endif	/* SENSOR_HPP */

