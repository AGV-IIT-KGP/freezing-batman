/*
 * File:   LogitechCamera.hpp
 * Author: satya
 *
 * Created on December 12, 2013, 8:49 PM
 */

#ifndef LOGITECHCAMERA_HPP
#define	LOGITECHCAMERA_HPP

#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <environment/Sensor.hpp>

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
    image_transport::Publisher publisher;
    int frame_id;
    cv::Mat frame;

    void loadParams(ros::NodeHandle& node_handle);
    void setupComms(ros::NodeHandle& node_handle);
};

#endif	/* LOGITECHCAMERA_HPP */
