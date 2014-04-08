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

extern IplImage *show_img1;
extern IplImage *show_img2;
extern IplImage *show_img3;
extern IplImage *show_img4;

class LaneDetector : public environment::Interpreter {
public:
    void interpret();
    LaneDetector();
    LaneDetector(const LaneDetector& orig);
    virtual ~LaneDetector();
    void getLanes(const sensor_msgs::ImageConstPtr& image);
    IplImage* colorBasedLaneDetection(IplImage* frame_in, int k);
    void applyHoughTransform(IplImage* img, IplImage *dst, int vote, int length, int merge);
    IplImage* joinResult(IplImage* color_gray, IplImage* hough_gray);
    void initializeLaneVariables(int argc, char** argv, ros::NodeHandle nh);
    CvSeq* GetHoughLanes(IplImage* img, int vote, int length, int mrgha);
    IplImage *getLaneLines(IplImage* src);
    image_transport::ImageTransport *getLaneNode();
private:
    CvSize size;
    int depth;
    IplImage *kernel_frame;
    IplImage *edge_frame;
    IplImage *gray_hough_frame;
    IplImage *gray_frame;
    IplImage *lane;
    IplImage *warp_img;
    IplImage* img;
    IplConvKernel *ker1;
    CvPoint offset;
    uchar** ImageData;
    uchar* data;
    double mean, std_dev;
    int i;
    int height, width;
    sensor_msgs::CvBridge bridge;
    CvPoint2D32f srcQuad[4], dstQuad[4];
    int canny_kernel, high_threshold, low_threshold, vote, length, mrg;
    int k;
    CvMat* warp_matrix;
    int mouseParam;
    image_transport::ImageTransport *it;
    void publishLanes(IplImage* final_img);

};

#endif	/* LANEDETECTOR_HPP */