/* 
 * File:   SituationSolver.cpp
 * Author: samuel
 * 
 * Created on 22 January, 2014, 1:06 PM
 */

#include <SituationSolver.hpp>
#include <utils/Pose2D.hpp>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>

SituationSolver::SituationSolver() {
    initialize();
}

SituationSolver::SituationSolver(const SituationSolver& orig) {
}

SituationSolver::~SituationSolver() {
}

nav_msgs::Path SituationSolver::nextManeuver() {
    nav_msgs::Path maneuver;
    for (unsigned int pose_id = 0; pose_id + 1 < waypoints.poses.size(); pose_id++) {
        Pose2D point1(waypoints.poses.at(pose_id).pose.position.x, 
                      waypoints.poses.at(pose_id).pose.position.y, 0);
        Pose2D point2(waypoints.poses.at(pose_id + 1).pose.position.x, 
                      waypoints.poses.at(pose_id + 1).pose.position.y, 0);
        
        for (double ix = point1.x; ix < point2.x; ix += interpolation_resolution) {
            Pose2D point3(ix, point1.y + (ix - point1.x) / (point2.x - point1.x)*(point2.y - point1.y), 0);
            Pose2D center(current_pose.position.x, current_pose.position.y, 0);
            if (sqrt(center.distance(point3)) < window_radius) {
                geometry_msgs::PoseStamped interpolated;
                interpolated.pose.position.x = point3.x;
                interpolated.pose.position.y = point3.y;
                interpolated.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(point2.y - point1.y, point2.x - point1.x));
                maneuver.poses.push_back(interpolated);
            }
        }
    }

    return maneuver;
}

void SituationSolver::initialize() {
    window_radius = 10; // Meters
    interpolation_resolution = .3; // Meters
}