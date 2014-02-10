/* 
 * File:   PathSegment.h
 * Author: Shiwangi
 *
 * Created on 23 January, 2014, 6:25 PM
 */

#ifndef PATHSEGMENT_H
#define	PATHSEGMENT_H

#include "State.h"
#include <vector>

class PathSegment {
public:
    PathSegment(){
    }
    PathSegment(std::vector<State>& path_, double lengthOfPath_) : path(path_), lengthOfPath(lengthOfPath_) {
    }
    std::vector<State> path;
    double lengthOfPath;
    double sigma;

};

#endif	/* PATHSEGMENT_H */

