/* 
 * File:   LaneDetector.hpp
 * Author: samuel
 *
 * Created on 14 December, 2013, 3:36 AM
 */

#ifndef LANEDETECTOR_HPP
#define	LANEDETECTOR_HPP

#include <environment/Interpreter.hpp>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <stdexcept>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

#define DEBUG 0
#define WAIT_TIME 1000
#define choice 2
#define scale 220/57
#define N 7 // canny kernel
#define AUTO_CALIB 0

#define MAP_MAX 1000
#define LOOP_RATE 10
#define WAIT_TIME 100

#define SCALE_X 10
#define MAP_MAX 1000

extern cv::Mat show_img1;
extern cv::Mat show_img2;
extern cv::Mat show_img3;
extern cv::Mat show_img4;

class LaneDetector : public environment::Interpreter {
public:
    void interpret();
    LaneDetector();
    LaneDetector(const LaneDetector& orig);
    virtual ~LaneDetector();
    void getLanes(const sensor_msgs::ImageConstPtr& image);
    cv::Mat colorBasedLaneDetection(cv::Mat frame_in, int k);
    void applyHoughTransform(cv::Mat img, cv::Mat dst, int vote, int length, int merge);
    cv::Mat joinResult(cv::Mat color_gray, cv::Mat hough_gray);
    void initializeLaneVariables(int argc, char** argv, ros::NodeHandle nh);
    std::vector<cv::Vec4i> GetHoughLanes(cv::Mat img, int vote, int length, int mrgha);
    cv::Mat getLaneLines(cv::Mat src);
    image_transport::ImageTransport getLaneNode();

private:
    cv::Size size;
    int depth;
    cv::Mat kernel_frame;
    cv::Mat edge_frame;
    cv::Mat gray_hough_frame;
    cv::Mat gray_frame;
    cv::Mat lane;
    cv::Mat warp_img;
    cv::Mat img;
    cv::Mat ker1;
    cv::Point offset;
    uchar** ImageData;
    uchar* data;
    double mean, std_dev;
    int i;
    int rows, cols;
    sensor_msgs::CvBridge bridge; /////??????????????????????????????? shouldn't it be a cv_bridge::CvImagePtr object
    cv::Point2f srcQuad[4], dstQuad[4];
    int canny_kernel, high_threshold, low_threshold, vote, length, mrg;
    int k;
    cv::Mat warp_matrix;
    int mouseParam;
    image_transport::ImageTransport image_transport;
    void publishLanes(cv::Mat final_img);
};

#endif	/* LANEDETECTOR_HPP */