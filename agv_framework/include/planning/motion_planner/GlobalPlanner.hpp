/* 
 * File:   GlobalPlanner.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 6:43 PM
 */

#ifndef GLOBALPLANNER_HPP
#define	GLOBALPLANNER_HPP

namespace planning {

    class GlobalPlanner {
    public:
        virtual void getNextWaypoint() = 0;
        GlobalPlanner();
        GlobalPlanner(const GlobalPlanner& orig);
        virtual ~GlobalPlanner();
    private:
    };
}
#endif	/* GLOBALPLANNER_HPP */

