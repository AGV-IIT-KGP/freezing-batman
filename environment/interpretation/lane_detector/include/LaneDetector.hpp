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
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <iostream>

class LaneDetector : public environment::Interpreter {
public:
    void interpret();
    LaneDetector();
    LaneDetector(const LaneDetector& orig);
    virtual ~LaneDetector();

    // cv::Mat detectLanes(cv::Mat);
    // cv::Mat preprocessing(cv::Mat);
    // cv::Mat background_seperation(cv::Mat);
    // cv::Mat thresholding(cv::Mat);
    // std::vector<cv::Mat> lane_seperation(cv::Mat);
    // cv::Mat curve_fitting(cv::Mat);
    // cv::Mat inverse_perspective_transform(cv::Mat);
    // cv::Mat merge_curves(std::vector<cv::Mat>);
    cv::Mat::rowRange
    void publishLanes(cv::Mat);
};


#endif	/* LANEDETECTOR_HPP */
