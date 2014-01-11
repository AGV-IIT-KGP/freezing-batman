/*
 * File:   LogitechCamera.hpp
 * Author: satya
 *
 * Created on December 12, 2013, 8:49 PM
 */

#ifndef LOGITECHCAMERA_HPP
#define	LOGITECHCAMERA_HPP

#include <environment/Sensor.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class LogitechCamera : public environment::Sensor {
public:
    bool connect();
    bool disconnect();
    bool fetch();
    void publish(int frame_id);

    LogitechCamera();
    LogitechCamera(int argc, char** argv);
    LogitechCamera(const LogitechCamera& orig);
    virtual ~LogitechCamera();
private:
    // Parameters
    int camera_id;
    std::string node_name;
    std::string topic_name;
    int message_queue_size;

    cv::VideoCapture capture;
    ros::NodeHandle node_handle;
    image_transport::Publisher publisher;
    int frame_id;
    cv::Mat frame;

    void initializeParameters();
    void initializeParameters(int argc, char** argv);
    void setupCommunications();
};

#endif	/* LOGITECHCAMERA_HPP */

