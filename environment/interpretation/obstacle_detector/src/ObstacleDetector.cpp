#include <cassert>
#include <climits>
#include <ObstacleDetector.hpp>
#include <opencv2/highgui/highgui_c.h>

void ObstacleDetector::publishData() {
    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = obstacle_map;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    obstacle_publisher.publish(out_msg.toImageMsg());
}

void ObstacleDetector::interpret() {
    if (debug_mode > 0) {
        cv::imshow(std::string("/") + node_name + std::string("/raw_scan"), obstacle_map);
        cv::waitKey(wait_time);
    }

    int dilation_size = obstacle_expansion;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                cv::Point(dilation_size, dilation_size));
    cv::dilate(obstacle_map, obstacle_map, element);
    publishData();

    if (debug_mode > 0) {
        cv::imshow(std::string("/") + node_name + std::string("/dilate_filter"), obstacle_map);
        cv::waitKey(wait_time);
    }
}

void ObstacleDetector::scanCallback(const sensor_msgs::LaserScan& scan) {
    size_t size = scan.ranges.size();
    float angle = scan.angle_min;
    float maxRangeForContainer = scan.range_max - 0.1f;
    obstacle_map = obstacle_map - obstacle_map; // Assign zero to all pixels

    for (size_t i = 0; i < size; ++i) {
        float dist = scan.ranges[i];
        if ((dist > scan.range_min) && (dist < maxRangeForContainer)) {
            double x1 = -1 * sin(angle) * dist;
            double y1 = cos(angle) * dist;
            int x = (int) ((x1 * 100) + center_x);
            int y = (int) ((y1 * 100) + center_y + lidar_y_shift);
            if (x >= 0 && y >= min_dist && (int) x < map_size && (int) y < max_dist) {
                int x2 = (x);
                int y2 = (map_size - y - 30 - 1);
                if (!(y2 >= 0 && y2 < map_size)) {
                    continue;
                }
                obstacle_map.at<uchar>(y2, x2) = 255;
            }
        }
        angle += scan.angle_increment;
    }

    interpret();
}

ObstacleDetector::ObstacleDetector(std::string node_name, ros::NodeHandle& node_handle) {
    loadParams(node_handle);
    this->node_name = node_name;
    
    if (debug_mode > 0) {
        cv::namedWindow(std::string("/") + node_name + std::string("/raw_scan"), CV_WINDOW_AUTOSIZE);
        cv::namedWindow(std::string("/") + node_name + std::string("/dilate_filter"), CV_WINDOW_AUTOSIZE);
    }

    obstacle_map = cv::Mat(map_size, map_size, CV_8UC1, cvScalarAll(0));
    image_transport = new image_transport::ImageTransport(node_handle);
    obstacle_publisher = image_transport->advertise(std::string("/") + node_name + std::string("/obstacles"), 10);
    scan_subscriber = node_handle.subscribe("/sensors/hokuyo_nodes/0", 2, &ObstacleDetector::scanCallback, this);
}

ObstacleDetector::~ObstacleDetector() {
}

void ObstacleDetector::loadParams(ros::NodeHandle& node_handle) {
    // Default Params used while debugging
    center_x = 500;
    center_y = 100;
    debug_mode = 0;
    hokuyo_scale = 100;
    lidar_y_shift = 30;
    map_size = 1000;
    max_dist = 400;
    min_dist = 0;
    obstacle_expansion = 30;
    wait_time = 100;

    node_handle.getParam(std::string("/") + node_name + std::string("/center_x"), center_x);
    node_handle.getParam(std::string("/") + node_name + std::string("/center_y"), center_y);
    node_handle.getParam(std::string("/") + node_name + std::string("/debug_mode"), debug_mode);
    node_handle.getParam(std::string("/") + node_name + std::string("/hokuyo_scale"), hokuyo_scale);
    node_handle.getParam(std::string("/") + node_name + std::string("/lidar_y_shift"), lidar_y_shift);
    node_handle.getParam(std::string("/") + node_name + std::string("/map_size"), map_size);
    node_handle.getParam(std::string("/") + node_name + std::string("/max_dist"), max_dist);
    node_handle.getParam(std::string("/") + node_name + std::string("/min_dist"), min_dist);
    node_handle.getParam(std::string("/") + node_name + std::string("/obstacle_expansion"), obstacle_expansion);
    node_handle.getParam(std::string("/") + node_name + std::string("/wait_time"), wait_time);
}

int main(int argc, char** argv) {
    std::string node_name("obstacle_detector");
    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle node_handle;

    ObstacleDetector obstacle_detector(node_name, node_handle);

    ros::spin();
    return 0;
}
