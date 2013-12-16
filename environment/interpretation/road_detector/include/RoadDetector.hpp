/* 
 * File:   RoadDetector.hpp
 * Author: samuel
 *
 * Created on 14 December, 2013, 1:33 AM
 */

#ifndef ROADDETECTOR_HPP
#define	ROADDETECTOR_HPP

#include <Environment/Interpreter.hpp>
#include <models/Road.hpp>

class RoadDetector : public environment::Interpreter {
public:
    void interpret();

    RoadDetector();
    RoadDetector(const RoadDetector& orig);
    virtual ~RoadDetector();
private:
    Road road;
};

#endif	/* ROADDETECTOR_HPP */

