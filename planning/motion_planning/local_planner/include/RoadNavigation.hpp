/* 
 * File:   RoadNavigation.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 7:37 PM
 */

#ifndef ROADNAVIGATION_HPP
#define	ROADNAVIGATION_HPP

#include <Header.hpp>
#include <Clothoid.hpp>

namespace navigation {

    class RoadNavigation {
    public:
        // State initialState,targetState;
        // std::vector<State> targetTraj;
        // std::vector<Path*> pathCollection;
        // Trajectory* trajectory;
        RoadNavigation();
        RoadNavigation(const RoadNavigation& orig);
        virtual ~RoadNavigation();
        nav_msgs::Path decideTargetTrajectory(nav_msgs::Path::ConstPtr lane_traj);
        double getDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
        std::vector<geometry_msgs::Pose> getTargets(geometry_msgs::Pose current_pose, nav_msgs::Path target_traj);
        std::vector<std::vector<PathSegment*> > getPaths(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> targets);
        void planRoadDetection(const nav_msgs::Path::ConstPtr& lane_traj, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
    };
}
#endif	/* ROADNAVIGATION_HPP */

