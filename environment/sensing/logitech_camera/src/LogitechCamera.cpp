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

#include <sstream>
#include <LifeCycle.hpp>
#include <LogitechCamera.hpp>

LogitechCamera::LogitechCamera(int argc, char** argv) : Sensor(argc, argv) {
    camera_id = std::atoi(argv[1]);
    node_name = argv[2];
    std::cout<<"camera_id:"<<camera_id<<std::endl;
    std::cout<<"node_name:"<<node_name<<std::endl;
    
    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle node_handle;
    loadParams(node_handle);
    setupComms(node_handle);
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
    return true;
}

void LogitechCamera::publish(int frame_id) {
    cv_bridge::CvImage message;
    message.header.seq = frame_id;
    message.header.frame_id = frame_id;
    message.header.stamp = ros::Time::now();
    message.encoding = sensor_msgs::image_encodings::BGR8;
    message.image = frame;
    publisher.publish(message.toImageMsg());
    imshow("window",frame);
    cv::waitKey(10);
}

void LogitechCamera::loadParams(ros::NodeHandle& node_handle) {
   // node_handle.getParam("/logitech_camera/camera_id", camera_id);
    //node_handle.getParam("/logitech_camera/node_name", node_name);
    //node_handle.getParam("/logitech_camera/publisher_queue_size", message_queue_size);
    std::ostringstream convert;
    convert << camera_id;
    std::cout<<"camera_id:"<<camera_id<<std::endl;
    std::cout<<"node_name:"<<node_name<<std::endl;
    topic_name = node_name + std::string("/image");
}

void LogitechCamera::setupComms(ros::NodeHandle& node_handle) {
    image_transport::ImageTransport image_transporter(node_handle);
    publisher = image_transporter.advertise(topic_name.c_str(), message_queue_size);
}

int main(int argc, char** argv) {
    double loop_rate;
    int frame_id;

    loop_rate = 10;
    frame_id = 0;

    LogitechCamera logitech_camera(argc, argv);
    logitech_camera.connect();

    ros::Rate rate_enforcer(loop_rate);
    while (ros::ok()) {
        logitech_camera.fetch();
        logitech_camera.publish(frame_id);
        frame_id++;
        ros::spinOnce();
        rate_enforcer.sleep();
    }

    logitech_camera.disconnect();
    return 0;
}
