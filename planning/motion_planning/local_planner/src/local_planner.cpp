/* 
 * File:   local_planner.cpp
 * Author: satya
 *
 * Created on December 13, 2013, 7:37 PM
 */

#include <road_navigation/RoadNavigation.hpp>
#include <geometry_msgs/PoseStamped.h>

using namespace message_filters;
ros::Publisher path_publisher;
using namespace navigation;

void planPath(const nav_msgs::Path::ConstPtr& lane_traj, const geometry_msgs::PoseStamped::ConstPtr& pose) {
    navigation::RoadNavigation planner;
    path_publisher.publish(planner.planRoadDetection(lane_traj, pose));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle node_handle;
    path_publisher = node_handle.advertise<nav_msgs::Path>("local_planner/path", 10);

    message_filters::Subscriber<nav_msgs::Path> lane_traj_sub(node_handle, "sensor_fusion/lanes/trajectory", 10);
    message_filters::Subscriber<geometry_msgs::PoseStamped> localization_pose_sub(node_handle, "localization/pose", 100);

    typedef sync_policies::ApproximateTime<nav_msgs::Path, geometry_msgs::PoseStamped> ApproxTimePolicy;
    Synchronizer<ApproxTimePolicy> sync(ApproxTimePolicy(100), lane_traj_sub, localization_pose_sub);

    sync.registerCallback(boost::bind(&planPath, _1, _2));

    ros::spin();

    return 0;
}
