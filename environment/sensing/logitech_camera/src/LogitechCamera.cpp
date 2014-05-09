/*
 * File:   LogitechCamera.cpp
 * Author: satya
 *
 * Created on December 12, 2013, 8:49 PM
 */

 /*
DOCUMENTATION:

Logitech Camera

======================================================

Description of Sensor 
------------------------------------------------------
Camera Module

Usage:
`rosrun logitech_camera logitech_camera <options(optional)>`

Description of Options
------------------------------------------------------
If options mentioned in usage
{
    camera_id = 0;
    node_name = std::string("camera");
    topic_name = std::string("sensors/camera");
    message_queue_size = 10;
}

Else
{
    camera_id = std::atoi(argv[1]);
    node_name = std::string("sensors_camera_") + std::string(argv[1]);
    topic_name = std::string("sensors/camera/") + std::string(argv[1]);
    message_queue_size = 10;
}

Publisher Data Type
-------------------------------------------------------
1.
Handle: "sensors/camera/" + (argv[1]);
Data Type: cv_bridge::CvImage message;


Other Relevant Information
-------------------------------------------------------
ros::Rate rate_enforcer(10)
*/


#include "LogitechCamera.hpp"
#include "LifeCycle.hpp"

// LogitechCamera::LogitechCamera() {
//     initializeParameters();
// 
//     int argc;
//     char** argv;
//     ros::init(argc, argv, node_name.c_str());
// 
//     setupCommunications();
// }

LogitechCamera::LogitechCamera(int argc, char** argv) : Sensor(argc, argv) {
    initializeParameters(argc, argv);

    ros::init(argc, argv, node_name.c_str());

    setupCommunications();
}

// LogitechCamera::LogitechCamera(const LogitechCamera& orig) :Sensor( ) {
//     ROS_INFO("q@QW%1ros::init has been called");
// }

LogitechCamera::~LogitechCamera() {
}

bool LogitechCamera::connect() {
    capture = cv::VideoCapture(camera_id);
    if (!capture.isOpened()) {
        // TODO: Raise specific exceptions
        ROS_WARN("Error while reading from camera");
        return false;
    }

    return true;
}

bool LogitechCamera::disconnect() {
    capture.release();

    // TODO: Raise specific exceptions
    return true;
}

bool LogitechCamera::fetch() {
    capture >> frame;
}

void LogitechCamera::publish(int frame_id) {
    cv_bridge::CvImage message;
    message.header.seq = frame_id;
    message.header.frame_id = frame_id;
    message.header.stamp = ros::Time::now();
    message.encoding = sensor_msgs::image_encodings::BGR8;
    message.image = frame;

    publisher.publish(message.toImageMsg());
}

void LogitechCamera::initializeParameters() {
    camera_id = 0;
    node_name = std::string("camera");
    topic_name = std::string("sensors/camera");
    message_queue_size = 10;
}

void LogitechCamera::initializeParameters(int argc, char** argv) {
    camera_id = std::atoi(argv[1]);
    node_name = std::string("sensors_camera_") + std::string(argv[1]);
    topic_name = std::string("sensors/camera/") + std::string(argv[1]);
    message_queue_size = 10;
}

void LogitechCamera::setupCommunications() {
    image_transport::ImageTransport image_transporter(node_handle);
    publisher = image_transporter.advertise(topic_name.c_str(), message_queue_size);
}

int main(int argc, char** argv) {
    double loop_rate;
    int frame_id;

    loop_rate = 10;
    frame_id = 0;

    LogitechCamera *logitech_camera = new LogitechCamera(argc, argv);
    logitech_camera->connect();
    ROS_INFO("Camera succesfully connected. \n");

    ros::Rate rate_enforcer(loop_rate);

    while (ros::ok()) {
        logitech_camera->fetch();
        logitech_camera->publish(frame_id);
        ros::spinOnce();
        ROS_INFO("New frame acquired. \n");
        rate_enforcer.sleep();
    }
    logitech_camera->disconnect();
return 0;
}
