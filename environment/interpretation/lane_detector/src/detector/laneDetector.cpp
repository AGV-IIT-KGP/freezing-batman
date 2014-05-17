#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <laneDetector.hpp>

LaneDetector::LaneDetector(ros::NodeHandle& node_handle) {
    loadParams(node_handle);

    // Grass Removal
    kernel_size = 8;
    svm = new SVM();
    svm->init(kernel_size * kernel_size * 3);
    std::string model_path = training_data_file + ".model";
    svm->loadModel(model_path.c_str());
    setupComms();

    if (debug_mode > 0) {
        original_image_window = ros::this_node::getName() + std::string("/original_image");
        cv::namedWindow(original_image_window, CV_WINDOW_AUTOSIZE);
        result_window = ros::this_node::getName() + std::string("/result");
        cv::namedWindow(result_window, CV_WINDOW_AUTOSIZE);
        grass_removal_output_window = ros::this_node::getName() + std::string("/grass_removal_output");
        cv::namedWindow(grass_removal_output_window, CV_WINDOW_AUTOSIZE);
        ipt_output_window = ros::this_node::getName() + std::string("/ipt_output");
        cv::namedWindow(ipt_output_window, CV_WINDOW_AUTOSIZE);
        obstacle_removal_output_window = ros::this_node::getName() + std::string("/obstacle_removal_output");
        cv::namedWindow(obstacle_removal_output_window, CV_WINDOW_AUTOSIZE);
        lane_binary_output = ros::this_node::getName() + std::string("/lane_binary_output");
        cv::namedWindow(lane_binary_output, CV_WINDOW_AUTOSIZE);
    }
}

LaneDetector::~LaneDetector() {
}

void LaneDetector::interpret() {
    total_time_elapsed = 0;
    cv::Mat result = original_image;
    if (debug_mode > 0) {
        cv::imshow(result_window, result);
        cv::waitKey(wait_time);
    }

    if (time_functions > 0) {
        gettimeofday(&tval_before, NULL);
    }
    result = grassRemoval(result);
    if (time_functions > 0) {
        gettimeofday(&tval_after, NULL);
        time_elapsed = tval_after.tv_sec + (tval_after.tv_usec / 1000000.0) - (tval_before.tv_sec + (tval_before.tv_usec / 1000000.0));
        total_time_elapsed += time_elapsed;
        if (time_functions == 2) {
            std::cout << "GrassRemoval FPS : " << 1. / time_elapsed << std::endl;
        }
    }
    if (debug_mode > 0) {
        cv::imshow(grass_removal_output_window, result);
        cv::waitKey(wait_time);
    }

    if (time_functions == 2) {
        gettimeofday(&tval_before, NULL);
    }
    result = inversePerspectiveTransform(result);
    if (time_functions == 2) {
        gettimeofday(&tval_after, NULL);
        time_elapsed = tval_after.tv_sec + (tval_after.tv_usec / 1000000.0) - (tval_before.tv_sec + (tval_before.tv_usec / 1000000.0));
        total_time_elapsed += time_elapsed;
        if (time_functions == 2) {
            std::cout << "InversePerspectiveTransform FPS : " << 1. / time_elapsed << std::endl;
        }
    }
    if (debug_mode > 0) {
        cv::imshow(ipt_output_window, result);
        cv::waitKey(wait_time);
    }

    if (time_functions > 0) {
        gettimeofday(&tval_before, NULL);
    }
    result = obstacleRemoval(result);
    if (time_functions > 0) {
        gettimeofday(&tval_after, NULL);
        time_elapsed = tval_after.tv_sec + (tval_after.tv_usec / 1000000.0) - (tval_before.tv_sec + (tval_before.tv_usec / 1000000.0));
        total_time_elapsed += time_elapsed;
        if (time_functions == 2) {
            std::cout << "ObstacleRemoval FPS : " << 1. / time_elapsed << std::endl;
        }
    }
    if (debug_mode > 0) {
        cv::imshow(obstacle_removal_output_window, result);
        cv::waitKey(wait_time);
    }

    if (time_functions > 0) {
        gettimeofday(&tval_before, NULL);
    }
    result = getLaneBinary(result);
    if (time_functions > 0) {
        gettimeofday(&tval_after, NULL);
        time_elapsed = tval_after.tv_sec + (tval_after.tv_usec / 1000000.0) - (tval_before.tv_sec + (tval_before.tv_usec / 1000000.0));
        total_time_elapsed += time_elapsed;
        if (time_functions == 2) {
            std::cout << "GetLaneBinary FPS : " << 1. / time_elapsed << std::endl;
        }
    }
    if (debug_mode > 0) {
        cv::imshow(lane_binary_output, result);
        cv::waitKey(wait_time);
    }

    if (time_functions > 0) {
        if (time_functions == 2) {
            std::cout << "Total FPS : " << 1. / total_time_elapsed << std::endl;
        } else {
            ROS_INFO("Total FPS : %lf", 1. / total_time_elapsed);
        }
    }

    publishLanes(result);
}

void LaneDetector::setupComms() {
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);
    lanes_publisher = image_transport.advertise(published_topic_name.c_str(), 2);
    image_subscriber = image_transport.subscribe(subscribed_topic_name, 2, &LaneDetector::detectLanes, this);
    std::cout << "Communications started with : " << std::endl
            << "\tSubscriber topic : " << subscribed_topic_name << std::endl
            << "\tPublisher topic  : " << published_topic_name << std::endl;
}

void LaneDetector::detectLanes(const sensor_msgs::ImageConstPtr& msg) {
    try {
        sensor_msgs::CvBridge bridge;
        original_image = bridge.imgMsgToCv(msg, "bgr8");
        ros::NodeHandle node_handle;
        node_handle.getParam("debug_mode", debug_mode);
        cv::waitKey(wait_time);
    } catch (sensor_msgs::CvBridgeException& e) {
        ROS_ERROR("Error in converting image");
    }

    if (debug_mode > 0) {
        cv::imshow(original_image_window, original_image);
        cv::waitKey(wait_time);
    }

    interpret();
}

void LaneDetector::publishLanes(cv::Mat &image) {
    cv_bridge::CvImage message;
    message.encoding = sensor_msgs::image_encodings::MONO8;
    if (debug_mode == 7) {
        message.image = cv::Mat::zeros(map_size, map_size, CV_8UC1);
        cv::resize(image, message.image, message.image.size());
    } else {
        message.image = image;
    }
    lanes_publisher.publish(message.toImageMsg());
}

void LaneDetector::loadParams(ros::NodeHandle& node_handle) {
    debug_mode = 5;
    ipt_offsets_file = std::string("../data/ipt_offsets0.txt");
    map_size = 1000;
    published_topic_name = std::string("/lane_detector0/lanes");
    subscribed_topic_name = std::string("/camera/image");
    time_functions = 0;
    training_data_file = std::string("../data/Samples");
    wait_time = 10;
    warp_matrix_file = std::string("../data/warp_matrix0.dat");
    
    std::string node_name = std::string("/") + ros::this_node::getName();
    node_handle.getParam(node_name + "/debug_mode", debug_mode);
    node_handle.getParam(node_name + "/ipt_offsets_file", ipt_offsets_file);
    node_handle.getParam(node_name + "/map_size", map_size);
    node_handle.getParam(node_name + "/published_topic_name", published_topic_name);
    node_handle.getParam(node_name + "/subscribed_topic_name", subscribed_topic_name);
    node_handle.getParam(node_name + "/time_functions", time_functions);
    node_handle.getParam(node_name + "/training_data_file", training_data_file);
    node_handle.getParam(node_name + "/wait_time", wait_time);
    node_handle.getParam(node_name + "/warp_matrix_file", warp_matrix_file);
}
