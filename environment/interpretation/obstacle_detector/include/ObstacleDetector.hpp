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

#define DEBUG 1

#define CENTERX 500
#define CENTERY 100
#define HOKUYO_SCALE 100
#define RADIUS 30
#define EXPAND_ITER 60


#define MAP_MAX 1000
#define LOOP_RATE 10
#define WAIT_TIME 100


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
};

#endif	/* OBSTACLEDETECTOR_HPP */
