/* 
 * File:   SimpleController.cpp
 * Author: samuel
 * 
 * Created on 28 January, 2014, 12:28 AM
 */

#include <control/SimpleController.hpp>
#include <std_msgs/Int64.h>

int count = 0;

SimpleController::SimpleController() {
}

SimpleController::SimpleController(ros::NodeHandle node_handle) {
    steering_publisher = node_handle.advertise<std_msgs::Int64>("interface/controls/steering", 100);
    brake_publisher = node_handle.advertise<std_msgs::Int64>("interface/controls/brake", 100);
    throttle_publisher = node_handle.advertise<std_msgs::Int64>("interface/controls/throttle", 100);
    encoder_subscriber = node_handle.subscribe("interface/state/encoder_counts", 1, &SimpleController::setEncoder_counts, this);
}

SimpleController::SimpleController(const SimpleController& orig) {
}

SimpleController::~SimpleController() {
}

void SimpleController::control() {
    // Steering: [-1000, 1000]
    // Brake: [-1000, 0]
    std_msgs::Int64 steering, brake, throttle;
    steering.data = 800;
    brake.data = -800;
    
    count++;
    if (count > 100) {
        count = 0;
    }
    throttle.data = count;

    //steering_publisher.publish(steering);
    //brake_publisher.publish(brake);
    throttle_publisher.publish(throttle);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_controller");
    ros::NodeHandle node_handle;

    SimpleController simple_controller(node_handle);

    ros::Rate loop_rate(2);
    while (ros::ok()) {
        simple_controller.control();

        loop_rate.sleep();
        ros::spinOnce();
    }
}