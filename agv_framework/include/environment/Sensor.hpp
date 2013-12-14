/* 
 * File:   Sensor.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 6:32 PM
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
        Sensor();
        Sensor(const Sensor& orig);
        virtual ~Sensor();


    };
}
#endif	/* SENSOR_HPP */

