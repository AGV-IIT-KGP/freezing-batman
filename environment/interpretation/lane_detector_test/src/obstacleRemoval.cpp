#include "laneDetector.hpp"

cv::Mat LaneDetector::ObstacleRemoval(cv::Mat &image){
	
    cv::Mat img_HSV(image.rows,image.cols,CV_8UC3);
    cv::Mat binary_after_HSV_thresholding(image.rows,image.cols,CV_8UC1);
    cv::Mat binary_dialated(image.rows,image.cols,CV_8UC1);
    
    cvtColor(image,img_HSV,CV_BGR2HSV);
    
    
    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 2*obstacle_removal_dilation_size + 1, 2*obstacle_removal_dilation_size+1 ),
                                       cv::Point( obstacle_removal_dilation_size, obstacle_removal_dilation_size ) );
    if (debug_mode==2) {
   	cv::namedWindow("HSV Control Box",1);
   	int h=0,s=0,v=0;
    cv::createTrackbar("lower hue value","HSV Control Box",&h,180,NULL);
    cv::createTrackbar("upper hue value","HSV Control Box",&s,255,NULL);
    cv::createTrackbar("lower saturation value","HSV Control Box",&v,255,NULL);
    do {
    	cv::inRange(img_HSV, cv::Scalar(h-1,v-1,-1), cv::Scalar(s,256,255), binary_after_HSV_thresholding); 
    	cv::imshow("HSV Control Box",binary_after_HSV_thresholding);
    	cv::dilate(binary_after_HSV_thresholding, binary_dialated, element);
    	if (cv::waitKey(33)==27) {
    		break;
    	}
   	} while (true); 
	
   } /*  (debug==2) */

    cv::inRange(img_HSV, cv::Scalar(obstacle_removal_hue,obstacle_removal_saturation,0), cv::Scalar(180,256,255), binary_after_HSV_thresholding); 
    cv::dilate(binary_after_HSV_thresholding, binary_dialated, element);
    
    for (int i=0;i<binary_dialated.rows;i++) {
        for (int j=0;j<binary_dialated.cols;j++) {
            if (binary_dialated.at<uchar>(i,j)==255) {
                image.at<cv::Vec3b>(i,j)[0]=image.at<cv::Vec3b>(i,j)[1]=image.at<cv::Vec3b>(i,j)[2]=0;
            }
        }
    }
    return image;
}


