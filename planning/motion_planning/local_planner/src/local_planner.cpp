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

void findPath(const nav_msgs::Path::ConstPtr& maneuver_ptr, 
              const geometry_msgs::PoseStamped::ConstPtr& pose_ptr,
              const nav_msgs::OccupancyGrid::ConstPtr& map_ptr) {
    navigation::RoadNavigation planner;
    path_publisher.publish(planner.plan(maneuver_ptr, pose_ptr, map_ptr));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle node_handle;
    path_publisher = node_handle.advertise<nav_msgs::Path>("local_planner/path", 10);

    message_filters::Subscriber<nav_msgs::Path> maneuver_subscriber(node_handle, "situational_planner/maneuver", 10);
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_subscriber(node_handle, "localization/pose", 100);
    // TODO: Replace this usage by a map server, which serves map snippets on demand.
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_subscriber(node_handle, "environment/map", 10);

    typedef sync_policies::ApproximateTime<nav_msgs::Path, geometry_msgs::PoseStamped, nav_msgs::OccupancyGrid> ApproxTimePolicy;
    Synchronizer<ApproxTimePolicy> synchronizer(ApproxTimePolicy(100), 
                                                maneuver_subscriber, 
                                                pose_subscriber, 
                                                map_subscriber);

    synchronizer.registerCallback(boost::bind(&findPath, _1, _2, _3));

    ros::spin();

    return 0;
}
