/* 
 * File:   RoadNavigation.hpp
 * Author: Shiwangi
 *
 * Created on December 13, 2013, 7:37 PM
 */

#ifndef ROADNAVIGATION_HPP
#define	ROADNAVIGATION_HPP

#include <Header.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "road_navigation/Clothoid.hpp"

namespace navigation {

    class RoadNavigation {
    public:
        RoadNavigation();
        RoadNavigation(const RoadNavigation& orig);
        virtual ~RoadNavigation();

        std::vector<nav_msgs::Path> getPaths() const {
            return paths;
        }

        nav_msgs::Path plan(nav_msgs::Path maneuver_ptr, geometry_msgs::PoseStamped pose_ptr, nav_msgs::OccupancyGrid map_ptr);

    private:
        double scale;
        bool debug;
        int num_targets;
        double spacing;
        double target_lookahead;
        std::string dt_output;
        std::string dt_input;
        std::string display;
        // TODO: Maintain a separate set of local maps, each having its own size and resolution
        //       It would be of type OccupancyGrid, provided by the agv_framework
        cv::Mat cost_map;
        std::vector<nav_msgs::Path> paths;

        double calculateTargetCost(nav_msgs::Path path);
        nav_msgs::Path convertToNavMsgsPath(Clothoid& trajectory);
        void setupObstacleCostMap(nav_msgs::OccupancyGrid map);
        void setupTargetCostMap(nav_msgs::Path target_trajectory, nav_msgs::OccupancyGrid map);
        void filterPaths(nav_msgs::OccupancyGrid map);
        double getDistance(geometry_msgs::Pose& pose1, geometry_msgs::Pose& pose2);
        void constructPaths(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> targets);
        std::vector<geometry_msgs::Pose> getTargets(geometry_msgs::Pose current_pose, nav_msgs::Path target_trajectory);
        bool pathIsFree(nav_msgs::Path path, nav_msgs::OccupancyGrid map);
    };
}
#endif	/* ROADNAVIGATION_HPP */

