#include <iostream>
#include <stdio.h>
#include <string.h>

#include <ros/ros.h>
#include <auro666_pilot/SetSteering.h>
#include <auro666_pilot/SetBrake.h>

#include <interface/roboteq/Constants.h>
#include <interface/roboteq/ErrorCodes.h>
#include <interface/roboteq/RoboteqDevice.h>

#define DEVICE_ID "/dev/serial/by-id/usb-Roboteq_Motor_Controller_498954A73235-if00"

using namespace std;

RoboteqDevice motor_controller;
int status = 0;

bool setSteering(auro666_pilot::SetSteering::Request &req, auro666_pilot::SetSteering::Response &res) {
    ROS_INFO("[interface/roboteq_server/setSteering] steering = %ld", (long int) req.steering);

    res.code = 0;

    if ((status = motor_controller.SetCommand(_GO, 1, req.steering)) != RQ_SUCCESS) {
        ROS_INFO("Failed... Error code --> %d", status);
    } else {
        ROS_DEBUG("Succeeded.");
        return true;
    }

    usleep(100);

    int numberOfattempts = 0;

    while (status != 0 && numberOfattempts < 10) {
        usleep(500000);
        ROS_INFO("Attempting server restart...");
        motor_controller.Disconnect();
        status = motor_controller.Connect(DEVICE_ID);
        if (status == 0) {
            ROS_INFO("Connection re-established...");
        }
        numberOfattempts++;
    }

    if (numberOfattempts == 10 && status != 0) {
        ROS_ERROR("Could not connect to Roboteq Motor Controller... Aborting operation...");
        res.code = -1;
    }

    //ROS_INFO("sending back response: [%ld]", (long int) res.code);

    return true;
}

bool setBrake(auro666_pilot::SetBrake::Request &req, auro666_pilot::SetBrake::Response &res) {
    ROS_INFO("[interface/roboteq_server/setBrake] brake = %ld", (long int) req.brake);

    res.code = 0;

    if ((status = motor_controller.SetCommand(_GO, 2, req.brake)) != RQ_SUCCESS) {
        ROS_INFO("Failed... Error code --> %d", status);
    } else {
        ROS_DEBUG("Succeeded.");
        return true;
    }

    usleep(100);

    int numberOfattempts = 0;

    while (status != 0 && numberOfattempts < 10) {
        usleep(500000);
        ROS_INFO("Attempting server restart...");
        motor_controller.Disconnect();
        status = motor_controller.Connect("/dev/serial/by-id/usb-Roboteq_Motor_Controller_498954A73235-if00");
        if (status == 0) {
            ROS_INFO("Connection re-established...");
        }
        numberOfattempts++;
    }

    if (numberOfattempts == 10 && status != 0) {
        ROS_ERROR("Could not connect to Roboteq Motor Controller... Aborting operation...");
        res.code = -1;
    }

    //ROS_INFO("sending back response: [%ld]", (long int) res.code);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "roboteq_server");
    ros::NodeHandle node_handle;

    string response = "";

    ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
    ROS_INFO("Initializing...");
    usleep(500000);

    status = motor_controller.Connect(DEVICE_ID);
    //status = device.Connect("/dev/ttyACM0");

    while (status != RQ_SUCCESS && ros::ok()) {
        ROS_INFO("Error connecting to device: %d", status);
        ROS_INFO("Attempting server restart...");
        usleep(999999);
        motor_controller.Disconnect();
        status = motor_controller.Connect(DEVICE_ID);
        if (status == 0) {
            ROS_INFO("Connection re-established...");
        }
    }

    ros::ServiceServer steering_service = node_handle.advertiseService("roboteq/steering", setSteering);
    ros::ServiceServer brake_service = node_handle.advertiseService("roboteq/brake", setBrake);
    ROS_INFO("Server initialized...");
    ROS_INFO("Ready to control motors...");

    ros::spin();

    motor_controller.Disconnect();

    return 0;
}
