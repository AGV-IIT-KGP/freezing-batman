/* 
 * File:   GSLTest.cpp
 * Author: samuel
 * 
 * Created on 24 January, 2014, 7:25 PM
 */

#include <curve_navigation/GSLTest.hpp>
#include <tf/transform_datatypes.h>
#include <constants.hpp>

GSLTest::GSLTest() {
}

GSLTest::GSLTest(ros::NodeHandle node_handle) {
    initialize();

    // TODO: Obtain this data from simulator and republish it
    map_msg.info.height = map_height;
    map_msg.info.width = map_width;
    map_msg.data.resize(map_height * map_width);
    std::fill(map_msg.data.begin(), map_msg.data.end(), 0);

    pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("localization/pose", 100);
    map_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>("environment/map", 10);

    state_publisher = node_handle.advertise<auro666_pilot::State>("vehicle_server/state", 100);
    pose_subscriber = node_handle.subscribe("simulator/pose", 2, &GSLTest::updatePose, this);
    state_subscriber = node_handle.subscribe("simulator/state", 2, &GSLTest::publishState, this);
}

GSLTest::GSLTest(const GSLTest& orig) {
}

GSLTest::~GSLTest() {
}

void GSLTest::updatePose(const geometry_msgs::Pose::ConstPtr& pose_ptr) {
    pose_msg.pose.position.x = pose_ptr->position.x;
    pose_msg.pose.position.y = pose_ptr->position.y;
    pose_msg.pose.orientation = pose_ptr->orientation;
}

void GSLTest::publishState(const auro666_pilot::State::ConstPtr& state_ptr) {
    state_publisher.publish(*state_ptr);
}

void GSLTest::sendTestData() {
    pose_msg.header.stamp = ros::Time::now();
    pose_publisher.publish(pose_msg);
    pose_msg.header.seq += 1;

    // TODO: Obtain this data from simulator and republish it
    map_msg.header.stamp = ros::Time::now();
    map_publisher.publish(map_msg);
    map_msg.header.seq += 1;
}

void GSLTest::initialize() {
    // In centimeters
    map_width = 6000;
    map_height = 3000;

    pose_msg.pose.position.x = 1; // Meters
    pose_msg.pose.position.y = 1;
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    pose_msg.header.seq = 0;
    map_msg.header.seq = 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gsl_tester");
    ros::NodeHandle node_handle;

    GSLTest gsl_tester(node_handle);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        gsl_tester.sendTestData();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}