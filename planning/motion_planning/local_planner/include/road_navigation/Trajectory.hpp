/* 
 * File:   Trajectory.hpp
 * Author: samuel
 *
 * Created on 21 December, 2013, 12:31 PM
 */

#ifndef TRAJECTORY_HPP
#define	TRAJECTORY_HPP

#include <ros/ros.h> // To be removed when debugging support is available in agv_framework
#include <cmath>
#include <iostream>
#include <vector>
#include "road_navigation/Clothoid.hpp"
#include "utils/Pose2D.hpp"

namespace navigation {

    class Trajectory {
    public:
        std::vector<Clothoid*> segments;

        Trajectory();
        Trajectory(const Trajectory& orig);
        virtual ~Trajectory();

        void setGoal(double x, double y, double theta);
        void setStart(double x, double y, double theta);

        void generate();

    private:
        Pose2D start, goal;

        void generateSegment(Pose2D& start, Pose2D& goal);
    };
}

#endif	/* TRAJECTORY_HPP */

