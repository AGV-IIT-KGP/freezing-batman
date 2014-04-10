#ifndef LANE_DATA_H
#define	LANE_DATA_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <stdexcept>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core_c.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>

#include <opencv2/highgui/highgui.hpp>
#include <environment/Interpreter.hpp>
#include <ros/ros.h>
using namespace cv;

class LaneDetection:public environment::Interpreter {
public:
	Mat gray_frame;
	Mat kernel_frame;
	Mat edge_frame;
        Mat gray_img;	
	Mat lane;
	sensor_msgs::CvBridge bridge;
	void markLane(const sensor_msgs::ImageConstPtr& image);
    	Mat colorBasedLaneDetection(Mat frame_in, int k);
     	Mat applyHoughTransform(Mat img,Mat dst, int vote, int length, int merge);
    	Mat joinResult(Mat color_gray,Mat hough_gray);
    	void initializeLaneVariables(Mat img);
	void interpret();
};
#endif

