/* 
 * File:   PathSegment.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 7:30 PM
 */

#ifndef PATHSEGMENT_HPP
#define	PATHSEGMENT_HPP

#include <iostream>
#include <vector>
#include <State.hpp>
namespace navigation {

    class PathSegment {
    public:
        PathSegment();
        PathSegment(const PathSegment& orig);
        virtual ~PathSegment();
        double larc;
        std::vector<State> sites;
    };
}

#endif	/* PATHSEGMENT_HPP */

