#include <LidarData.h>

char **laser_scan;

int main(int argc, char** argv) {
    std::string node_name("indoor_obstacle_detector");
    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle lidar_node;
    LidarData lidar_data;

    image_transport::ImageTransport image_transport(lidar_node);
    lidar_data.obstacle_publisher = image_transport.advertise(std::string("/") + node_name + std::string("/obstacles"), 10);
    ros::Subscriber scan_subscriber = lidar_node.subscribe("/sensors/hokuyo_nodes/0", 2, &LidarData::updateMap, &lidar_data);
    //ros::Subscriber scan_subscriber = lidar_node.subscribe("/scan", 2, &LidarData::update_map, &lidardata);
    cv::Mat show_image1(400, 400, CV_8UC1, cvScalarAll(0));
    cv::Mat show_image2(400, 400, CV_8UC1, cvScalarAll(0));
    cv::Mat show_image3(400, 400, CV_8UC1, cvScalarAll(0));
    ros::Rate loop_rate(10);

    bool fps_test = false;

    int iterations = 0;
    time_t start = time(0);
    if (fps_test) {
        ROS_INFO("[LIDAR] Conducting an FPS Test");
    }

    while (ros::ok()) {
        if (fps_test) {
            if (iterations > 100) {
                time_t finish = time(0);
                double fps = (iterations + 0.0) / (finish - start);
                ROS_INFO("[LIDAR] FPS: %lf", fps);
                break;
            }
            iterations++;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}