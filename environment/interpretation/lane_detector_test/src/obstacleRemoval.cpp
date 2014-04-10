#include "../include/laneDetector.hpp"

static int obstacle_removal_dilation_size = 30;     //variable used for dilating and eroding.. to be changed only if dimension of image changes.
static int obstacle_removal_hue = 25;               //used to remove obstacle, change only after calibration.
static int obstacle_removal_saturation = 100;       //used to remove obstacle, change only after calibration.

cv::Mat LaneDetector::ObstacleRemoval(cv::Mat &image){

    cv::Mat img_HSV(image.rows, image.cols, CV_8UC3);
    cv::Mat binary_after_HSV_thresholding(image.rows, image.cols, CV_8UC1);
    cv::Mat binary_dialated(image.rows, image.cols, CV_8UC1);

    cvtColor(image, img_HSV, CV_BGR2HSV);

    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 2*obstacle_removal_dilation_size + 1, 2*obstacle_removal_dilation_size+1 ),
                                       cv::Point( obstacle_removal_dilation_size, obstacle_removal_dilation_size ) );
    if (debug_mode==2) {
        std::string file_path = data_path + "/obstacle.dat";

        FILE* obstacle_removal_threshold_file = fopen(file_path.c_str(), "w");

        cv::namedWindow("HSV Control Box",1);
        cv::createTrackbar("Dilation element size","HSV Control Box", &obstacle_removal_dilation_size, 180,NULL);
        cv::createTrackbar("Hue Threshold","HSV Control Box", &obstacle_removal_hue, 255,NULL);
        cv::createTrackbar("Saturation Threshold","HSV Control Box", &obstacle_removal_saturation, 255,NULL);

        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                               cv::Size( 2*obstacle_removal_dilation_size + 1, 2*obstacle_removal_dilation_size+1 ),
                               cv::Point( obstacle_removal_dilation_size, obstacle_removal_dilation_size ) );

        cv::inRange(img_HSV, cv::Scalar(0, obstacle_removal_saturation,0), cv::Scalar(obstacle_removal_hue,256,255), binary_after_HSV_thresholding);
        cv::dilate(binary_after_HSV_thresholding, binary_dialated, element);

        for (int i=0;i<binary_dialated.rows;i++) {
            for (int j=0;j<binary_dialated.cols;j++) {
                if (binary_dialated.at<uchar>(i,j)==255) {
                    image.at<cv::Vec3b>(i,j)[0]=image.at<cv::Vec3b>(i,j)[1]=image.at<cv::Vec3b>(i,j)[2]=0;
                }
            }
        }

        cv::imshow("HSV Control Box", image);

        fprintf(obstacle_removal_threshold_file, "%d %d %d", obstacle_removal_dilation_size, obstacle_removal_hue, obstacle_removal_saturation);

        fclose(obstacle_removal_threshold_file);

        return image;

    } /*  (debug==2) */

    cv::inRange(img_HSV, cv::Scalar(0, obstacle_removal_saturation,0), cv::Scalar(obstacle_removal_hue,256,255), binary_after_HSV_thresholding);
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

void ReadParameterFromFile(){
    static bool read_parameters = false;
    if(!read_parameters){
        std::cout << "Reading Obstacle Removal Parameters from file" << std::endl;
    }
    return;
}
