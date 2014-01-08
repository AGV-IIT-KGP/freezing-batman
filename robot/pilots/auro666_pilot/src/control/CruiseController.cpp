/* 
 * File:   CruiseControl.cpp
 * Author: samuel
 * 
 * Created on 3 January, 2014, 3:56 PM
 */

#include <control/CruiseController.hpp>

CruiseController::CruiseController() {
    path_ended = false;
}

CruiseController::CruiseController(const CruiseController& orig) {
}

CruiseController::~CruiseController() {
}

float CruiseController::getCruiseControl() {
    pathEndCheck();

    float cmd_rear_wheel_speed;

    if (path.poses.size() == 0 || path_ended) {
        cmd_rear_wheel_speed = 0;
    } else {
        cmd_rear_wheel_speed = 1;
    }

    return cmd_rear_wheel_speed;
}

double CruiseController::displacement(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2) {
    return sqrt((pose1.position.x - pose2.position.x) * (pose1.position.x - pose2.position.x) +
                (pose1.position.y - pose2.position.y) * (pose1.position.y - pose2.position.y));
}

geometry_msgs::Pose CruiseController::obtainSteerPoint(geometry_msgs::Pose pose) {
    geometry_msgs::Pose steer_point = pose;
    double theta = tf::getYaw(pose.orientation);
    steer_point.position.x += distance * cos(theta);
    steer_point.position.y += distance * sin(theta);

    return steer_point;
}

void CruiseController::pathEndCheck() {
    geometry_msgs::Pose steer_point = obtainSteerPoint(pose);

    if (path.poses.size() > 0) {
        double min_distance = displacement(steer_point, path.poses.at(0).pose);
        unsigned int closest_pose_id = 0;
        for (unsigned int pose_id = 0; pose_id < path.poses.size(); pose_id++) {
            double distance = displacement(steer_point, path.poses.at(pose_id).pose);
            if (distance < min_distance) {
                min_distance = distance;
                closest_pose_id = pose_id;
            }
        }

        if (closest_pose_id + 1 == path.poses.size()) {
            path_ended = true;
        }
    }
}