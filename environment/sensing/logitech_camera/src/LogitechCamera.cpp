/* 
 * File:   LogitechCamera.cpp
 * Author: satya
 * 
 * Created on December 12, 2013, 8:49 PM
 */

#include "LogitechCamera.hpp"
#include "LifeCycle.hpp"

LogitechCamera::LogitechCamera() {
    initializeParameters();

    int argc;
    char** argv;
    ros::init(argc, argv, node_name.c_str());

    setupCommunications();
}

LogitechCamera::LogitechCamera(int argc, char** argv) {
    initializeParameters(argc, argv);

    ros::init(argc, argv, node_name.c_str());

    setupCommunications();
}

LogitechCamera::LogitechCamera(const LogitechCamera& orig) {
}

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

    LogitechCamera logitech_camera = LogitechCamera(argc, argv);

    logitech_camera.connect();

    ros::Rate rate_enforcer(loop_rate);

    while (ros::ok()) {
        logitech_camera.fetch();
        logitech_camera.publish(frame_id);
        ros::spinOnce();
        rate_enforcer.sleep();
    }

    logitech_camera.disconnect();
}
