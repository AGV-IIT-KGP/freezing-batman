/* 
 * File:   SimpleController.hpp
 * Author: samuel
 *
 * Created on 28 January, 2014, 12:28 AM
 */

#ifndef SIMPLECONTROLLER_HPP
#define	SIMPLECONTROLLER_HPP

#include <ros/ros.h>
#include <std_msgs/Int64.h>

class SimpleController {
public:
    SimpleController();
    SimpleController(ros::NodeHandle node_handle);
    SimpleController(const SimpleController& orig);
    virtual ~SimpleController();

    void setEncoder_counts(std_msgs::Int64 msg) {
        this->encoder_counts = (int) msg.data;
    }

    void control();

private:
    int encoder_counts;

    ros::Publisher steering_publisher;
    ros::Publisher brake_publisher;
    ros::Publisher throttle_publisher;
    ros::Subscriber encoder_subscriber;
};

#endif	/* SIMPLECONTROLLER_HPP */

