#ifndef OBSTACLEDETECTOR_HPP
#define	OBSTACLEDETECTOR_HPP

#include <environment/Interpreter.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <stdexcept>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/LaserScan.h"

static const int DEBUG =1 ;



// SCALING  : 100 in cm.

static const int CENTERX =500;
static const int CENTERY =100;
static const int HOKUYO_SCALE= 100;
// static const int RADIUS =30;
static const int EXPAND_OBS =30;
static const int LIDAR_Y_SHIFT=30;

static const int MAP_MAX =1000;
static const int LOOP_RATE =10;
static const int WAIT_TIME =100;


class ObstacleDetector : public environment::Interpreter {
public:
    void interpret();
    ObstacleDetector(int argc, char *argv[],ros::NodeHandle &nh);
    virtual ~ObstacleDetector();
private:
	cv::Mat img;
	void publishData();
	void scanCallback(const sensor_msgs::LaserScan& scan);
	//void initialize(int argc, char* argc[]);
    std::string topic_name,sub_topic_name;
	ros::NodeHandle nh;
	ros::Subscriber sub;
    image_transport::ImageTransport *it;
    image_transport::Publisher pub;
    int max_dist;
    int min_dist;
};

#endif	/* OBSTACLEDETECTOR_HPP */
