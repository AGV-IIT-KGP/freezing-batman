/* 
 * File:   Posture.cpp
 * Author: samuel
 * 
 * Created on 21 December, 2013, 12:33 PM
 */

#include "road_navigation/Posture.hpp"

namespace navigation {

    Posture::Posture(double x, double y, double theta, double curvature) {
        this->x = x;
        this->y = y;
        this->theta = theta;
        this->curvature = curvature;
    }

    Posture::Posture() {
    }

    Posture::Posture(const Posture& orig) {
    }

    Posture::~Posture() {
    }
}

