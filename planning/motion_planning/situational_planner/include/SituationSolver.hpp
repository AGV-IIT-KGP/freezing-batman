/* 
 * File:   SituationSolver.hpp
 * Author: samuel
 *
 * Created on 22 January, 2014, 1:06 PM
 */

#ifndef SITUATIONSOLVER_HPP
#define	SITUATIONSOLVER_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class SituationSolver {
public:
    SituationSolver();
    SituationSolver(const SituationSolver& orig);
    virtual ~SituationSolver();

    void SetCurrent_pose(geometry_msgs::PoseStamped current_pose) {
        this->current_pose = current_pose.pose;
    }
    
    void SetWaypoints(nav_msgs::Path waypoints) {
        this->waypoints = waypoints;
    }

    nav_msgs::Path nextManeuver();

private:
    double window_radius;
    double interpolation_resolution;
    geometry_msgs::Pose current_pose;
    nav_msgs::Path waypoints;
    
    void initialize();
};

#endif	/* SITUATIONSOLVER_HPP */

