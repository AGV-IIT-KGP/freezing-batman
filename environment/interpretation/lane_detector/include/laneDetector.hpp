#ifndef _LANE_DETECTOR_HPP_
#define _LANE_DETECTOR_HPP_

#include <iostream>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <environment/Interpreter.hpp>
#include <libsvm/svmWrapper.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

class LaneDetector : public environment::Interpreter {
public:
    LaneDetector(ros::NodeHandle& node_handle);
    ~LaneDetector();

    void interpret();

private:
    int debug_mode;
    int time_functions;
    int wait_time;
    int map_size;
    std::string subscribed_topic_name, published_topic_name;
    std::string training_data_file, ipt_offsets_file, warp_matrix_file;
    std::string original_image_window, result_window, grass_removal_output_window, ipt_output_window, obstacle_removal_output_window, lane_binary_output;

    image_transport::Publisher lanes_publisher;
    image_transport::Subscriber image_subscriber;
    
    ros::Publisher cloud_pub;

    cv::Mat original_image; // Raw image

    struct timeval tval_before, tval_after;
    double time_elapsed;
    double total_time_elapsed;

    // Grass Removal
    SVM *svm;
    int kernel_size;

    void loadParams(ros::NodeHandle& node_handle);

    // Image Processing Functions
    cv::Mat preprocessing(cv::Mat &image); // Image enhancement functions
    cv::Mat shadowRemoval(cv::Mat &image); //ShadowRemoval applied
    cv::Mat grassRemoval(cv::Mat &image); // Apply grass removal and return the image with grass removed
    cv::Mat obstacleRemoval(cv::Mat &image); // Remove Obstacles and return the image with obstacles removed
    cv::Mat getLaneBinary(cv::Mat &image); // Detect lanes and return a binary image with Lanes only
    cv::Mat seperateLanes(cv::Mat &image); // Seperate the binary image into different lanes
    cv::Mat fixBrokenLanes(cv::Mat &image); // Curve Fitting and Dilate to fix the broken lanes
    cv::Mat inversePerspectiveTransform(cv::Mat &image); // Change the view to bird's eye view
    pcl::PointCloud<pcl::PointXYZ>::Ptr generatecloud(cv::Mat &image);

    // Communication Functions
    void detectLanes(const sensor_msgs::ImageConstPtr& msg); // Publish the Lane Binary Image
    void publishLanes(cv::Mat &image); // Publish the Lane Binary Image
    void setupComms(); // Set up ros communication
};

#endif /* _LANE_DETECTOR_HPP_ */
