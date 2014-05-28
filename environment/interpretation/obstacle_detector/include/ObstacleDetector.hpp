#ifndef OBSTACLEDETECTOR_HPP
#define	OBSTACLEDETECTOR_HPP

#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <environment/Interpreter.hpp>

// SCALING  : 100 in cm.

class ObstacleDetector : public environment::Interpreter {
public:
    void interpret();
    ObstacleDetector(std::string node_name, ros::NodeHandle& node_handle);
    virtual ~ObstacleDetector();
private:
    int debug_mode;
    std::string node_name;
    std::string obstacles_topic_name;
    std::string scan_topic_name;
    
    int center_x, center_y;
    int hokuyo_scale;
    int obstacle_expansion;
    int lidar_y_shift;
    int map_size;
    int max_dist, min_dist;
    int wait_time;

    cv::Mat obstacle_map;
    ros::Subscriber scan_subscriber;
    image_transport::ImageTransport *image_transport;
    image_transport::Publisher obstacle_publisher;

    void loadParams(ros::NodeHandle& node_handle);
    void publishData();
    void scanCallback(const sensor_msgs::LaserScan& scan);
};

#endif	/* OBSTACLEDETECTOR_HPP */
