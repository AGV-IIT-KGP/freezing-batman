#include "laneDetector.hpp"

#include <stdexcept>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

cv::Mat LaneDetector::GrassRemoval(cv::Mat &image){
	svm->predictKernelWise(image, kernel_size);
return image;	
}
