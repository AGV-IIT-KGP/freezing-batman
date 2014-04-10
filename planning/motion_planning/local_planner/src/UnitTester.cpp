/* 
 * File:   RoadNavigationTester.cpp
 * Author: samuel
 * 
 * Created on 25 December, 2013, 8:20 PM
 */

#include <UnitTester.hpp>

UnitTester::UnitTester() {
}

UnitTester::UnitTester(ros::NodeHandle node_handle) {
    map_width = 1000;
    map_height = 1000;
    num_samples = 1000;
    scale = .01;
    pi = 3.14;

    map_msg.info.height = map_height;
    map_msg.info.width = map_width;
    map_msg.data.resize(map_height * map_width);
    std::fill(map_msg.data.begin(), map_msg.data.end(), 0);

    maneuver_msg.poses.resize(num_samples);
    for (unsigned int pose_id = 0; pose_id < maneuver_msg.poses.size(); pose_id++) {
        maneuver_msg.poses.at(pose_id).pose.position.x = pose_id * map_width / num_samples * scale;
        maneuver_msg.poses.at(pose_id).pose.position.y = pose_id * map_height / num_samples * scale;
        maneuver_msg.poses.at(pose_id).pose.orientation = tf::createQuaternionMsgFromYaw(pi / 4);
    }
    
    pose_msg.pose.position.x = map_width / 2 * scale;
    pose_msg.pose.position.y = map_height / 10 * scale;
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(pi / 2);

    pose_msg.header.seq = 0;
    maneuver_msg.header.seq = 0;
    map_msg.header.seq = 0;

    maneuver_publisher = node_handle.advertise<nav_msgs::Path>("situational_planner/maneuver", 10);
    pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("localization/pose", 100);
    map_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>("environment/map", 100);
}

UnitTester::UnitTester(const UnitTester& orig) {
}

UnitTester::~UnitTester() {
}

void UnitTester::sendTestData() {
    maneuver_msg.header.stamp = ros::Time::now();
    maneuver_publisher.publish(maneuver_msg);
    maneuver_msg.header.seq += 1;

    map_msg.header.stamp = ros::Time::now();
    map_publisher.publish(map_msg);
    map_msg.header.seq += 1;

    pose_msg.header.stamp = ros::Time::now();
    pose_publisher.publish(pose_msg);
    pose_msg.header.seq += 1;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner_unit_tester");
    ros::NodeHandle node_handle;
    UnitTester tester(node_handle);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        tester.sendTestData();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}