/* 
 * File:   LaneDetector.hpp
 * Author: samuel
 *
 * Created on 14 December, 2013, 3:36 AM
 */

#ifndef LANEDETECTOR_HPP
#define	LANEDETECTOR_HPP

#include <Environment/Interpreter.hpp>

class LaneDetector : public environment::Interpreter {
public:
    void interpret();
    
    LaneDetector();
    LaneDetector(const LaneDetector& orig);
    virtual ~LaneDetector();
private:

};

#endif	/* LANEDETECTOR_HPP */

