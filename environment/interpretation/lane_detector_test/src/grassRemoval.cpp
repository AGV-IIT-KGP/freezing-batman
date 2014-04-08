#include "laneDetector.hpp"

#include <stdexcept>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

cv::Mat LaneDetector::GrassRemoval(cv::Mat &image){
	cv::Mat tempImage = image.clone();
	cv::Mat imgRoi;
	int result=99;
	int start_row, start_col;
	for (int i=0;i<image.rows;i+=kernel_size/2) {
		for (int j=0;j<image.cols;j+=kernel_size/2) {
			
			if ( (image.rows-i-1) > kernel_size && (image.cols-j-1) > kernel_size ) {
				start_col = j;
				start_row = i;
			}
			else if ( (image.rows-i-1) > kernel_size ) {
				start_col = image.cols - kernel_size - 1;
				start_row = i;
			}
			else if ( (image.cols-j-1) > kernel_size ) {
				start_col = j;
				start_row = image.rows - kernel_size - 1;
			}
			else{
				start_col = image.cols - kernel_size - 1;
				start_row = image.rows - kernel_size - 1;
			}
			
			imgRoi = tempImage(cv::Rect(start_col, start_row, kernel_size, kernel_size));
			svm->predict(imgRoi, result);
			if ( result == 1 ) {
				cv::rectangle( image, cv::Point( start_col, start_row ), cv::Point( start_col+kernel_size, start_row+kernel_size ), cv::Scalar( 0, 0, 0 ),CV_FILLED);
			}
		}
	}
return image;	
}
