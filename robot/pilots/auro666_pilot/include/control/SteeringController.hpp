/* 
 * File:   SteeringController.hpp
 * Author: samuel
 *
 * Created on 3 January, 2014, 2:34 PM
 */

#ifndef STEERINGCONTROLLER_HPP
#define	STEERINGCONTROLLER_HPP

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <auro666_pilot/State.h>

class SteeringController {
public:
    SteeringController();
    SteeringController(const SteeringController& orig);
    virtual ~SteeringController();

    double GetCte() const {
        return cte;
    }

    void SetState(const auro666_pilot::State::ConstPtr state_ptr) {
        state = *state_ptr;
    }

    void SetPath(const nav_msgs::Path::ConstPtr path_ptr) {
        path = *path_ptr;
        path_ended = false;
    }

    void SetPose(const geometry_msgs::Pose pose) {
        this->pose = pose;
    }

    float getSteeringControl();

private:
    static const double pgain = 1;
    static const double igain = 0;
    static const double dgain = 0;
    // In meters
    static const double distance = 0.6;

    double cte;
    double cte_sum;
    double cte_last;
    double delta_steer_angle;
    double path_orientation;
    double current_vehicle_pose;
    double c; //landing Curve Gain which decides the convergence of cte and curvature of landing curve

    geometry_msgs::Pose pose;
    nav_msgs::Path path;
    auro666_pilot::State state;
    bool path_ended;

    unsigned int calculateClosestPoseId(geometry_msgs::Pose steer_point);
    void calculateParams();
    double displacement(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
    int sgn(double d);
    geometry_msgs::Pose obtainSteerPoint(geometry_msgs::Pose pose);
};

#endif	/* STEERINGCONTROLLER_HPP */

