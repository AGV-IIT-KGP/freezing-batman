#include <cstdio>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

extern image_transport::Publisher world_map_publisher;
void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& lidar, const sensor_msgs::ImageConstPtr& image2);
void singleCallback(const sensor_msgs::ImageConstPtr& image);
