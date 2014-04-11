#ifndef _LANE_DETECTOR_HPP_
#define _LANE_DETECTOR_HPP_

#include <ros/ros.h>
#include <iostream>
#include <environment/Interpreter.hpp>
#include <image_transport/image_transport.h>

#include <stdexcept>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <opencv/cvwimage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "libsvm/svmWrapper.hpp"

#define WAIT_TIME 10
#define MAP_MAX 1000

class LaneDetector : public environment::Interpreter {
private:

	int debug_mode;
	int timeFunctions;
	std::string sub_topic_name, pub_topic_name;
	ros::NodeHandle nh_;

    std::string data_path;

	cv_bridge::CvImage cvi;
	sensor_msgs::CvBridge bridge;
	image_transport::ImageTransport it;
	image_transport::Publisher pub;
	image_transport::Subscriber sub;

	cv::Mat Image; // Raw image

	struct timeval tvalBefore, tvalAfter;
	double timeElapsed;
	double totalTimeElapsed;
	
	// Grass Removal
	SVM *svm;
	int kernel_size;
	
	// Image Processing Functions
	cv::Mat Preprocessing(cv::Mat &image);   // Image enhancement functions
	cv::Mat GrassRemoval(cv::Mat &image);    // Apply grass removal and return the image with grass removed
	cv::Mat ObstacleRemoval(cv::Mat &image); // Remove Obstacles and return the image with obstacles removed
	cv::Mat GetLaneBinary(cv::Mat &image);   // Detect lanes and return a binary image with Lanes only
	cv::Mat SeperateLanes(cv::Mat &image);   // Seperate the binary image into different lanes
	cv::Mat FixBrokenLanes(cv::Mat &image);  // Curve Fitting and Dilate to fix the broken lanes
	cv::Mat InversePerspectiveTransform(cv::Mat &image);  // Change the view to bird's eye view

	// Communication Functions
	void SubscribeImage(const sensor_msgs::ImageConstPtr& msg);       // Publish the Lane Binary Image
	void PublishLanes(cv::Mat &image);       // Publish the Lane Binary Image
	void setUpCommunication(); // Set up ros communication

public:
	LaneDetector(std::string _pub_topic_name, std::string _sub_topic_name, int _timeFunctions = 0, int _debugMode = 0);
	~LaneDetector();
	void interpret();
};

#endif /* _LANE_DETECTOR_HPP_ */
