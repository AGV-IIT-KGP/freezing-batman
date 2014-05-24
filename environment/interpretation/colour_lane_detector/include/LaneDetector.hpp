#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <stdexcept>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core_c.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/highgui.h>

#ifndef LANE_DATA_H
#define	LANE_DATA_H

#define MAP_MAX 1000
#define WAIT_TIME 10

IplImage *show_img1;
IplImage *show_img2;
IplImage *show_img3;
IplImage *show_img4;

class LaneDetector {
private:
    std::string subscribed_topic_name, published_topic_name;
    image_transport::Publisher lanes_publisher;
    image_transport::Subscriber image_subscriber;

    void loadParams(ros::NodeHandle node_handle);
    void setupComms(ros::NodeHandle node_handle);
    void populateLanes(IplImage *img);
public:
    LaneDetector(ros::NodeHandle node_handle);
    void markLane(const sensor_msgs::ImageConstPtr& image);
    IplImage* colorBasedLaneDetection(IplImage* frame_in);
    IplImage* applyHoughTransform(IplImage* img);
    IplImage* joinResult(IplImage* color_gray, IplImage* hough_gray);
    void initializeLaneVariables(IplImage *img);
    
};

#endif