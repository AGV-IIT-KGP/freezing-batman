/* 
 * File:   UnitTester.cpp
 * Author: samuel
 * 
 * Created on 7 January, 2014, 7:30 PM
 */

#include <control/UnitTester.hpp>

UnitTester::UnitTester() {
}

UnitTester::UnitTester(ros::NodeHandle& node_handle) {
    path_size = 1000;
    path_type = 1;

    pose_msg.pose.position.x = 1; // Meters
    pose_msg.pose.position.y = 1;
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    pose_msg.header.seq = 0;

    // Mimics other nodes
    state_publisher = node_handle.advertise<auro666_pilot::State>("vehicle_server/state", 100);
    pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("localization/pose", 100);
    path_publisher = node_handle.advertise<nav_msgs::Path>("local_planner/path", 10);

    // Observed stuff from simulator. Noise can be added later.
    state_subscriber = node_handle.subscribe("simulator/state", 2, &UnitTester::publishState, this);
    pose_subscriber = node_handle.subscribe("simulator/pose", 2, &UnitTester::publishPose, this);

    selectPath();
}

UnitTester::UnitTester(const UnitTester& orig) {
}

UnitTester::~UnitTester() {
}

void UnitTester::publishPose(const geometry_msgs::Pose::ConstPtr& pose_ptr) {
    pose_msg.pose.position.x = pose_ptr->position.x;
    pose_msg.pose.position.y = pose_ptr->position.y;
    pose_msg.pose.orientation = pose_ptr->orientation;
}

void UnitTester::publishState(const auro666_pilot::State::ConstPtr& state) {
    state_publisher.publish(*state);
}

void UnitTester::selectPath() {
    path.poses.resize(path_size);
    for (unsigned int pose_id = 0; pose_id < path.poses.size(); pose_id++) {
        path.poses[pose_id].pose = pathGenerator(pose_id);
    }
}

void UnitTester::sendTestData(int frame_id) {
    pose_msg.header.stamp = ros::Time::now();
    pose_publisher.publish(pose_msg);
    pose_msg.header.seq = frame_id;

    path.header.stamp = ros::Time::now();
    path_publisher.publish(path);
    path.header.seq = frame_id;
    path.header.frame_id = frame_id;
}

geometry_msgs::Pose UnitTester::pathGenerator(unsigned int pose_id) {
    geometry_msgs::Pose pose;

    switch (path_type) {
        case 0:
            pose.position.x = pose_id;
            pose.position.y = pose_id;
            pose.orientation = tf::createQuaternionMsgFromYaw(PI / 4);
            break;
        case 1:
            pose.position.x = pose_id;
            pose.position.y = sin(pose_id / 100.) * 100;
            pose.orientation = tf::createQuaternionMsgFromYaw(atan(cos(pose_id / 100.)));
            break;
    }

    return pose;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_unit_tester");
    ros::NodeHandle node_handle;

    UnitTester unit_tester(node_handle);

    ros::Rate loop_rate(10);
    int frame_id = 0;
    while (ros::ok()) {
        unit_tester.sendTestData(frame_id);

        ros::spinOnce();
        loop_rate.sleep();
        ++frame_id;
    }

    return 0;
}