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
        path.poses.resize(path_ptr->poses.size());
        for (unsigned int pose_id = 0; pose_id < path_ptr->poses.size(); pose_id++) {
            path.poses.at(pose_id).pose.position.x = path_ptr->poses.at(pose_id).pose.position.x / 100;
            path.poses.at(pose_id).pose.position.y = path_ptr->poses.at(pose_id).pose.position.y / 100;
            path.poses.at(pose_id).pose.orientation = path_ptr->poses.at(pose_id).pose.orientation;
        }
        path_ended = false;
    }

    void SetPose(const geometry_msgs::Pose pose) {
        this->pose.position.x = pose.position.x / 100;
        this->pose.position.y = pose.position.y / 100;
        this->pose.orientation = pose.orientation;        
    }

    float getCruiseControl();    

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

