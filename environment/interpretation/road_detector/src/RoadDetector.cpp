/* 
 * File:   RoadDetector.cpp
 * Author: samuel
 * 
 * Created on 14 December, 2013, 1:33 AM
 */

#include "RoadDetector.hpp"

void RoadDetector::interpret() {
}

/**
 * Constructor has several overrides each corresponding to unique ways of building road models.
 * As an example, one can have a road model interpreted by a single camera.
 * A similar (but, much well informed) road model can be interpreted by using a 
 * combination of a lidar and camera. 
 * The two of these methods will be implemented in separate constructors.
 */
RoadDetector::RoadDetector() {
}

RoadDetector::RoadDetector(const RoadDetector& orig) {
}

RoadDetector::~RoadDetector() {
}

int main(int argc, char** argv) {
    return 0;
}
