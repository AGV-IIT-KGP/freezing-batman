#include "data_fuser/fusion.h"

using namespace sensor_msgs;
using namespace message_filters;

float x_offset = -24;
float y_offset = -7;
int x_off = 500, y_off = 500;

void on_trackbar(int, void*) {
    x_offset = 500 - x_off;
    y_offset = 500 - y_off;
    std::cout << x_offset << " " << y_offset << std::endl;
}

void callback(const ImageConstPtr& lidar_image, const ImageConstPtr& lane_image) {
    cv_bridge::CvImagePtr lane_map;
    cv_bridge::CvImagePtr lidar_map;

    try {
        lane_map = cv_bridge::toCvCopy(lane_image, image_encodings::MONO8);
        lidar_map = cv_bridge::toCvCopy(lidar_image, image_encodings::MONO8);
        cv::Mat fusion_map = cv::Mat::zeros(lane_map->image.size(), CV_8UC1); //commented for calibration
        /*cv::Mat fusion_map=lane_map->image;  /////// added later for calibration
        cv::namedWindow("fusion",1);    
                cv::createTrackbar("x_offset- move left to decrease","fusion",&x_off,fusion_map.rows,on_trackbar);
                cv::createTrackbar("y_offset- move left to decrease","fusion",&y_off,fusion_map.cols,on_trackbar);
                cv::imshow("fusion",fusion_map);
                cv::waitKey(10);*/
        for (int i = 0; i < fusion_map.rows; i++) {
            for (int j = 0; j < fusion_map.cols; j++) {
                if ((i + x_offset >= 0 && i + x_offset < fusion_map.rows && j + y_offset >= 0 && j + y_offset < fusion_map.cols && lidar_map->image.at<uchar>(i + x_offset, j + y_offset) == 255) || lane_map->image.at<uchar>(i + x_offset, j) == 255) {
                    fusion_map.at<uchar>(i, j) = 255;
                } else {
                    fusion_map.at<uchar>(i, j) = 0;
                }
            }
        }

        int dilation_size = 11;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                    cv::Point(dilation_size, dilation_size));
        cv::dilate(fusion_map, fusion_map, element);

        cv_bridge::CvImage message;
        message.encoding = image_encodings::MONO8;
        message.image = fusion_map;
        world_map_publisher.publish(message.toImageMsg());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void singleCallback(const ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(image, image_encodings::MONO8);
        cv_bridge::CvImage message;
        message.encoding = image_encodings::MONO8;
        message.image = cv_image->image;
        world_map_publisher.publish(message.toImageMsg());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}
