/* 
 * File:   GlobalPlanner.hpp
 * Author: Satya Prakash
 *
 * Created on December 12, 2013, 11:24 PM
 */

 
#ifndef GLOBALPLANNER_HPP
#define	GLOBALPLANNER_HPP

namespace planning {
    
    class GlobalPlanner {
    public:
        virtual void getNextWaypoint() = 0;
    };
}

#endif	/* GLOBALPLANNER_HPP */

