/* 
 * File:   RoadNavigation.cpp
 * Author: satya
 * 
 * Created on December 13, 2013, 7:37 PM
 */

// TODO: Speed up calculations by using a lower resolution for the DT maps.

#include <road_navigation/RoadNavigation.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>

namespace navigation {

    RoadNavigation::RoadNavigation() {
        scale = 100;

        num_targets = 20;
        // In centimeters
        spacing = 10;
        target_lookahead = 400;
        dt_input = std::string("DT Input");
        dt_output = std::string("DT Output");
        display = std::string("Display");
        debug = false;

        if (debug) {
            cv::namedWindow(dt_output.c_str(), 0);
            cv::namedWindow(dt_input.c_str(), 0);
            cv::namedWindow(display.c_str(), 0);
        }
    }

    RoadNavigation::RoadNavigation(const RoadNavigation& orig) {
    }

    RoadNavigation::~RoadNavigation() {
    }

    nav_msgs::Path RoadNavigation::plan(nav_msgs::Path maneuver_ptr, geometry_msgs::PoseStamped pose_ptr, nav_msgs::OccupancyGrid map_ptr) {
        nav_msgs::Path target_trajectory = maneuver_ptr;
        geometry_msgs::Pose current_pose = pose_ptr.pose;
        nav_msgs::OccupancyGrid map = map_ptr;

        // TODO: This class shouldn't bother about this conversion. Do it outside.
        for (unsigned int pose_id = 0; pose_id < target_trajectory.poses.size(); pose_id++) {
            target_trajectory.poses.at(pose_id).pose.position.x *= scale;
            target_trajectory.poses.at(pose_id).pose.position.y *= scale;
        }
        current_pose.position.x *= scale;
        current_pose.position.y *= scale;

        std::vector<geometry_msgs::Pose> targets = getTargets(current_pose, target_trajectory);
        constructPaths(current_pose, targets);
        //filterPaths(map);
        //setupObstacleCostMap(map);
        //setupTargetCostMap(target_trajectory, map);

        nav_msgs::Path best_path;
        if (paths.size() != 0) {
            // TODO: Costs to include:
            //       1. Obstacle cost
            //       2. Steering angle cost
            //            best_path = paths.at(0);
            //            double min_cost = calculateTargetCost(paths.at(0));
            //            for (unsigned int path_id = 0; path_id < paths.size(); path_id++) {
            //                double cost = calculateTargetCost(paths.at(path_id));
            //                if (cost < min_cost) {
            //                    min_cost = cost;
            //                    best_path = paths.at(path_id);
            //                }
            //            }
            best_path = paths.at(paths.size() / 2);
        } else {
            ROS_WARN("[local_planner/RoadNavigation/planRoadDetection] No path found");
        }

        return best_path;
    }

    double RoadNavigation::calculateTargetCost(nav_msgs::Path path) {
        double cost_sum = 0;
        for (unsigned int i = 0; i < path.poses.size(); i++) {
            cost_sum += cost_map.at<float>(cv::Point((int) path.poses.at(i).pose.position.x,
                                                     (int) path.poses.at(i).pose.position.y));
        }
        double cost = cost_sum / (path.poses.size() + 1);
        ROS_DEBUG("[local_planner/RoadNavigation/calculateTargetCost] cost = %lf", cost);
        return cost;
    }

    nav_msgs::Path RoadNavigation::convertToNavMsgsPath(Trajectory& trajectory) {
        nav_msgs::Path path;
        for (unsigned int segment_id = 0; segment_id < trajectory.segments.size(); segment_id++) {
            for (unsigned int point_id = 0; point_id < trajectory.segments.at(segment_id)->points.size(); point_id++) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = trajectory.segments.at(segment_id)->points.at(point_id)->x;
                pose.pose.position.y = trajectory.segments.at(segment_id)->points.at(point_id)->y;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(trajectory.segments.at(segment_id)->points.at(point_id)->theta);
                path.poses.push_back(pose);
            }
        }

        return path;
    }

    void RoadNavigation::setupObstacleCostMap(nav_msgs::OccupancyGrid map) {

    }

    void RoadNavigation::setupTargetCostMap(nav_msgs::Path target_trajectory, nav_msgs::OccupancyGrid map) {
        cv::Mat input(map.info.height, map.info.width, CV_8U, cv::Scalar(255));

        for (unsigned int pose_id = 0; pose_id + 1 < target_trajectory.poses.size(); pose_id++) {
            cv::line(input,
                     cv::Point(target_trajectory.poses.at(pose_id).pose.position.x,
                               target_trajectory.poses.at(pose_id).pose.position.y),
                     cv::Point(target_trajectory.poses.at(pose_id + 1).pose.position.x,
                               target_trajectory.poses.at(pose_id + 1).pose.position.y),
                     cv::Scalar(0), 3, CV_AA, 0);
        }

        cv::distanceTransform(input, cost_map, CV_DIST_L2, 5);

        if (debug) {
            cv::imshow(dt_input.c_str(), input);
            cv::waitKey(10);

            cv::Mat output;
            cv::normalize(cost_map, output, 0, 1.0, cv::NORM_MINMAX);
            cv::imshow(dt_output.c_str(), output);
            cv::waitKey(10);
        }
    }

    void RoadNavigation::filterPaths(nav_msgs::OccupancyGrid map) {
        std::vector<nav_msgs::Path> filtered_paths;

        // TODO: Prune against kinematic and dynamic constraints

        for (unsigned int i = 0; i < paths.size(); i++) {
            if (pathIsFree(paths.at(i), map)) {
                filtered_paths.push_back(paths.at(i));
            }
        }

        paths.clear();
        for (unsigned int i = 0; i < filtered_paths.size(); i++) {
            paths.push_back(filtered_paths.at(i));
        }
    }

    double RoadNavigation::getDistance(geometry_msgs::Pose& pose1,
                                       geometry_msgs::Pose& pose2) {
        return sqrt(pow(pose1.position.x - pose2.position.x, 2) + pow(pose1.position.y - pose2.position.y, 2));
    }

    void RoadNavigation::constructPaths(geometry_msgs::Pose current_pose,
                                        std::vector<geometry_msgs::Pose> targets) {
        for (unsigned int target_id = 0; target_id < targets.size(); target_id++) {
            Trajectory trajectory;
            trajectory.setStart(current_pose.position.x, current_pose.position.y, tf::getYaw(current_pose.orientation));
            trajectory.setGoal(targets.at(target_id).position.x, targets.at(target_id).position.y, tf::getYaw(targets.at(target_id).orientation));
            trajectory.generate();
            paths.push_back(convertToNavMsgsPath(trajectory));
        }
    }

    std::vector<geometry_msgs::Pose> RoadNavigation::getTargets(geometry_msgs::Pose current_pose,
                                                                nav_msgs::Path target_trajectory) {
        int closest_target_pose_id = -1;
        if (target_trajectory.poses.size() != 0) {
            float min_distance = getDistance(current_pose, target_trajectory.poses.at(0).pose);
            for (unsigned int pose_id = 0; pose_id < target_trajectory.poses.size(); pose_id++) {
                float distance = getDistance(current_pose, target_trajectory.poses.at(pose_id).pose);
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_target_pose_id = pose_id;
                }
            }
        }
        ROS_DEBUG("[local_planner/RoadNavigation/getTargets] closest_target_pose_index = %d", closest_target_pose_id);

        std::vector<geometry_msgs::Pose> targets;

        if ((closest_target_pose_id < 0) || (closest_target_pose_id >= (int) target_trajectory.poses.size())) {
            return targets;
        }

        int center_target_pose_id = closest_target_pose_id;
        double distance_along_target_trajectory = 0;
        while (center_target_pose_id + 1 < (int) target_trajectory.poses.size() &&
               distance_along_target_trajectory < target_lookahead) {
            distance_along_target_trajectory += getDistance(target_trajectory.poses.at(center_target_pose_id).pose,
                                                            target_trajectory.poses.at(center_target_pose_id + 1).pose);
            center_target_pose_id++;
        }
        ROS_DEBUG("[local_planner/RoadNavigation/getTargets] center_target_index = %d", center_target_pose_id);
        ROS_DEBUG("[local_planner/RoadNavigation/getTargets] center_target = (%lf, %lf, %lf)",
                  target_trajectory.poses.at(center_target_pose_id).pose.position.x,
                  target_trajectory.poses.at(center_target_pose_id).pose.position.y,
                  tf::getYaw(target_trajectory.poses.at(center_target_pose_id).pose.orientation));

        double yaw = tf::getYaw(target_trajectory.poses.at(center_target_pose_id).pose.orientation);
        for (int target_id = -num_targets / 2; target_id <= num_targets / 2; target_id++) {
            geometry_msgs::Pose target = target_trajectory.poses.at(center_target_pose_id).pose;
            target.position.x = target_trajectory.poses.at(center_target_pose_id).pose.position.x + target_id * spacing * sin(yaw);
            target.position.y = target_trajectory.poses.at(center_target_pose_id).pose.position.y - target_id * spacing * cos(yaw);
            targets.push_back(target);
        }

        if (debug) {
            cv::Mat targets_display(1000, 1000, CV_8U, cv::Scalar(0));
            for (unsigned int i = 0; i < targets.size(); i++) {
                cv::circle(targets_display,
                           cv::Point(targets.at(i).position.x,
                                     targets.at(i).position.y),
                           3, cv::Scalar(255), 1, CV_AA, 0);
            }
            cv::circle(targets_display,
                       cv::Point(targets.at(num_targets / 2).position.x,
                                 targets.at(num_targets / 2).position.y),
                       5, cv::Scalar(255), 3, CV_AA, 0);
            cv::imshow(display.c_str(), targets_display);
            cv::waitKey(10);
        }

        return targets;
    }

    bool RoadNavigation::pathIsFree(nav_msgs::Path path, nav_msgs::OccupancyGrid map) {
        bool pass = true;

        for (unsigned int pose_id = 0; pose_id < path.poses.size(); pose_id++) {
            int x = (int) path.poses.at(pose_id).pose.position.x;
            int y = (int) path.poses.at(pose_id).pose.position.y;
            int map_cell_id = y * map.info.width + x;
            if ((0 <= map_cell_id) && (map_cell_id < (int) map.data.size())) {
                if (map.data.at(map_cell_id) > 50) {
                    pass = false;
                }
            } else {
                pass = false;
            }
        }

        return pass;
    }
}
