/* 
 * File:   LidarData.h
 * Author: bhuvnesh
 *
 * Created on 18 September, 2012, 6:56 PM
 */
//#include "../../eklavya2.h"
#include <opencv2/core/types_c.h>
#include "sensor_msgs/LaserScan.h"
#include <fstream>
 #include <iostream>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <environment/Interpreter.hpp>



using namespace std;

#ifndef LIDARDATA_H
#define	LIDARDATA_H

class LidarData {
public:
     cv::Mat img;
	 image_transport::Publisher obstacle_publisher;
     LidarData();
     void update_map(const sensor_msgs::LaserScan&);
     void publishData(cv::Mat img);
     static void writeVal(int val);
    virtual ~LidarData();
     

private:
    static void createCircle(int x, int y);
	
};

#endif	/* LIDARDATA_H */
