/* 
 * File:   AStarPlanner.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 12:54 AM
 */

#ifndef ASTARPLANNER_HPP
#define	ASTARPLANNER_HPP

#include <GlobalPlanner.hpp>

class AStarPlanner : public GlobalPlanner {
public:
    AStarPlanner();
    AStarPlanner(const AStarPlanner& orig);
    virtual ~AStarPlanner();
    void getNextWaypoint();
private:

};

#endif	/* ASTARPLANNER_HPP */

