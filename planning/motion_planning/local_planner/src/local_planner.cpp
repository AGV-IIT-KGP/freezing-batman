/* 
 * File:   local_planner.cpp
 * Author: satya
 *
 * Created on December 13, 2013, 7:37 PM
 */

// TODO: Maintain planner state which includes map, target trajectories and current pose.
//       Each subscriber can update the data independently. 
//       Synchronization can be achieved by spinOnce().
//       Planner is called regularly in a loop.

#include <road_navigation/RoadNavigation.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace message_filters;
ros::Publisher path_publisher;

void planPath(const nav_msgs::Path::ConstPtr& lane_traj_ptr, 
              const geometry_msgs::PoseStamped::ConstPtr& pose_ptr,
              const nav_msgs::OccupancyGrid::ConstPtr& map_ptr) {
    navigation::RoadNavigation planner;
    path_publisher.publish(planner.planRoadDetection(lane_traj_ptr, pose_ptr, map_ptr));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle node_handle;
    path_publisher = node_handle.advertise<nav_msgs::Path>("local_planner/path", 10);

    message_filters::Subscriber<nav_msgs::Path> lane_traj_sub(node_handle, "sensor_fusion/lanes/trajectory", 10);
    message_filters::Subscriber<geometry_msgs::PoseStamped> localization_pose_sub(node_handle, "localization/pose", 100);
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub(node_handle, "slam/map", 10);

    typedef sync_policies::ApproximateTime<nav_msgs::Path, geometry_msgs::PoseStamped, nav_msgs::OccupancyGrid> ApproxTimePolicy;
    Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), lane_traj_sub, localization_pose_sub, map_sub);

    sync.registerCallback(boost::bind(&planPath, _1, _2, _3));

    ros::spin();

    return 0;
}
