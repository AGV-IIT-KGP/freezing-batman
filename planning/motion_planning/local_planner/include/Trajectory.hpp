/* 
 * File:   Trajectory.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 7:34 PM
 */

#ifndef TRAJECTORY_HPP
#define	TRAJECTORY_HPP


#include <iostream>
#include <vector>
#include <PathSegment.hpp>
#include <State.hpp>
#include <Header.hpp>
namespace navigation {

    class Trajectory {
    public:
        Trajectory();
        Trajectory(const Trajectory& orig);
        virtual ~Trajectory();
        double larc;

        virtual std::vector<PathSegment*> drawPath(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose)=0;
  
    };
}


#endif	/* TRAJECTORY_HPP */

