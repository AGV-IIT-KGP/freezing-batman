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
    public:
        virtual bool connect() = 0;
        virtual bool fetch() = 0;
        virtual bool disconnect() = 0;
        virtual void publish(int frame_id) = 0;
        // Sensor();
        Sensor(int argc, char **argv);
        Sensor(const Sensor& orig);
        virtual ~Sensor();
    };
}
#endif	/* SENSOR_HPP */

