/* 
 * File:   CruiseControl.hpp
 * Author: samuel
 *
 * Created on 3 January, 2014, 3:56 PM
 */

#ifndef CRUISECONTROL_HPP
#define	CRUISECONTROL_HPP

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

class CruiseController {
public:
    CruiseController();
    CruiseController(const CruiseController& orig);
    virtual ~CruiseController();

    void SetPath(const nav_msgs::Path::ConstPtr path_ptr) {
        path = *path_ptr;
    }

    float getCruiseControl();

    void SetPose(const geometry_msgs::Pose::ConstPtr pose_ptr) {
        pose = *pose_ptr;
        path_ended = false;
    }


private:
    // TODO: This value must be borrowed from auro666_pilot class
    static const double distance = 0.6;

    nav_msgs::Path path;
    geometry_msgs::Pose pose;
    bool path_ended;

    double displacement(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
    geometry_msgs::Pose obtainSteerPoint(geometry_msgs::Pose pose);
    void pathEndCheck();
};

#endif	/* CRUISECONTROL_HPP */

