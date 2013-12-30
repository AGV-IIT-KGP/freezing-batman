/* 
 * File:   RoadNavigation.cpp
 * Author: satya
 * 
 * Created on December 13, 2013, 7:37 PM
 */

#include <road_navigation/RoadNavigation.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>

namespace navigation {

    RoadNavigation::RoadNavigation() {
        spacing = 10;
        num_targets = 20;
        target_lookahead = 400;
    }

    RoadNavigation::RoadNavigation(const RoadNavigation& orig) {
    }

    RoadNavigation::~RoadNavigation() {
    }

    nav_msgs::Path RoadNavigation::planRoadDetection(const nav_msgs::Path::ConstPtr& lane_traj_ptr,
                                                     const geometry_msgs::PoseStamped::ConstPtr& pose_ptr,
                                                     const nav_msgs::OccupancyGrid::ConstPtr& map_ptr) {
        nav_msgs::Path target_traj = decideTargetTrajectory(lane_traj_ptr);
        geometry_msgs::Pose current_pose = pose_ptr->pose;
        nav_msgs::OccupancyGrid map = *map_ptr;

        std::vector<geometry_msgs::Pose> targets = getTargets(current_pose, target_traj);
        std::vector<nav_msgs::Path> paths = getPaths(current_pose, targets);
        paths = filterPaths(paths, map);

        nav_msgs::Path best_path;
        if (paths.size() != 0) {
            // TODO: Choose the best path
            for (int i = 0; i < paths.size(); i++) {
            }
            best_path = paths.at(paths.size() / 2);
        } else {
            ROS_WARN("[local_planner/RoadNavigation/planRoadDetection] No path found");
        }

        return best_path;
    }

    nav_msgs::Path RoadNavigation::convertToNavMsgsPath(Trajectory& trajectory) {
        nav_msgs::Path path;
        for (int segment_id = 0; segment_id < trajectory.segments.size(); segment_id++) {
            for (int point_id = 0; point_id < trajectory.segments.at(segment_id)->points.size(); point_id++) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = trajectory.segments.at(segment_id)->points.at(point_id)->x;
                pose.pose.position.y = trajectory.segments.at(segment_id)->points.at(point_id)->y;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(trajectory.segments.at(segment_id)->points.at(point_id)->theta);
                path.poses.push_back(pose);
            }
        }

        return path;
    }

    nav_msgs::Path RoadNavigation::decideTargetTrajectory(nav_msgs::Path::ConstPtr lane_traj) {
        return *lane_traj;
    }

    std::vector<nav_msgs::Path> RoadNavigation::filterPaths(std::vector<nav_msgs::Path> paths,
                                                            nav_msgs::OccupancyGrid map) {
        std::vector<nav_msgs::Path> filtered_paths;

        // TODO: Prune against kinematic and dynamic constraints

        for (int i = 0; i < paths.size(); i++) {
            if (pathIsFree(paths.at(i), map)) {
                filtered_paths.push_back(paths.at(i));
            }
        }

        return filtered_paths;
    }

    double RoadNavigation::getDistance(geometry_msgs::Pose& pose1,
                                       geometry_msgs::Pose& pose2) {
        return sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2));
    }

    std::vector<nav_msgs::Path> RoadNavigation::getPaths(geometry_msgs::Pose current_pose,
                                                         std::vector<geometry_msgs::Pose> targets) {
        std::vector<nav_msgs::Path> paths;

        for (int target_id = 0; target_id < targets.size(); target_id++) {
            Trajectory trajectory;
            trajectory.setStart(current_pose.position.x, current_pose.position.y, tf::getYaw(current_pose.orientation));
            trajectory.setGoal(targets.at(target_id).position.x, targets.at(target_id).position.y, tf::getYaw(targets.at(target_id).orientation));
            trajectory.generate();
            paths.push_back(convertToNavMsgsPath(trajectory));
        }

        return paths;
    }

    std::vector<geometry_msgs::Pose> RoadNavigation::getTargets(geometry_msgs::Pose current_pose,
                                                                nav_msgs::Path target_traj) {
        int closest_target_pose_index = 0;
        float min_distance = getDistance(current_pose, target_traj.poses.at(0).pose);
        for (int i = 0; i < target_traj.poses.size(); i++) {
            float distance = getDistance(current_pose, target_traj.poses.at(i).pose);
            if (distance < min_distance) {
                min_distance = distance;
                closest_target_pose_index = i;
            }
        }
        ROS_DEBUG("[local_planner/RoadNavigation/getTargets] closest_target_pose_index = %d", closest_target_pose_index);

        int center_target_index = closest_target_pose_index;
        double distance_along_target_trajectory = 0;
        while (center_target_index + 1 < target_traj.poses.size() &&
               distance_along_target_trajectory < target_lookahead) {
            distance_along_target_trajectory += getDistance(target_traj.poses.at(center_target_index).pose,
                                                            target_traj.poses.at(center_target_index + 1).pose);
            center_target_index++;
        }

        ROS_DEBUG("[local_planner/RoadNavigation/getTargets] center_target_index = %d", center_target_index);
        ROS_DEBUG("[local_planner/RoadNavigation/getTargets] target = (%lf, %lf)",
                  target_traj.poses[center_target_index].pose.position.x,
                  target_traj.poses[center_target_index].pose.position.y);

        std::vector<geometry_msgs::Pose> targets;
        double slope = atan((current_pose.position.y - target_traj.poses[center_target_index].pose.position.y) / (current_pose.position.x - target_traj.poses[center_target_index].pose.position.x));
        for (int target_id = -num_targets / 2; target_id <= num_targets / 2; target_id++) {
            geometry_msgs::Pose target = target_traj.poses[center_target_index].pose;
            target.position.x = target_traj.poses[center_target_index].pose.position.x + target_id * spacing * sin(slope);
            target.position.y = target_traj.poses[center_target_index].pose.position.y - target_id * spacing * cos(slope);
            targets.push_back(target);
        }

        return targets;
    }

    bool RoadNavigation::pathIsFree(nav_msgs::Path path, nav_msgs::OccupancyGrid map) {
        bool pass = true;

        for (int pose_index = 0; pose_index < path.poses.size(); pose_index++) {
            int x = (int) path.poses.at(pose_index).pose.position.x;
            int y = (int) path.poses.at(pose_index).pose.position.y;
            int map_index = y * map.info.width + x;
            if ((0 <= map_index) && (map_index < map.data.size())) {
                if (map.data.at(map_index) > 50) {
                    pass = false;
                }
            } else {
                pass = false;
            }
        }

        return pass;
    }
}
