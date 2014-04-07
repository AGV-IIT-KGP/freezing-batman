#include <iostream>
#include <stdio.h>

#include <ros/ros.h>
#include <eklavya_odometry/odometry.h>

using namespace std;

odometry_space::OdometryFactory *odometry_factory;

void encoderCallback(const eklavya_encoder::Encoder_Data::ConstPtr& msg) {

    ROS_INFO("Encoder data received...");

    odometry_factory->updateOdometryData(msg);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "odometry");

    odometry_factory = new odometry_space::OdometryFactory();

    ros::NodeHandle n;
    ros::Publisher odometry_publisher = n.advertise<nav_msgs::Odometry > ("odom", 30, true);
    ros::Subscriber encoder_subscriber = n.subscribe("encoder", 1, encoderCallback);

    nav_msgs::Odometry odometry_message;

    ros::Rate publisher_rate(10);

    printf("Odometry node initialized...\n");

    while (ros::ok()) {

        ros::spinOnce();

        odometry_message = odometry_factory->getOdometryData();

        odometry_publisher.publish(odometry_message);

        publisher_rate.sleep();

    }

    return 0;
}
