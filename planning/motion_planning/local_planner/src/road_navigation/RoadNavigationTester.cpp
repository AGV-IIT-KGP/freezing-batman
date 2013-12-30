/* 
 * File:   RoadNavigationTester.cpp
 * Author: samuel
 * 
 * Created on 25 December, 2013, 8:20 PM
 */

#include <road_navigation/RoadNavigationTester.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

RoadNavigationTester::RoadNavigationTester() {
    map_width = 1000;
    map_height = 1000;
    num_samples = 1000;
    scale = .5;
    pi = 3.14;
    window_name = "RoadNavigationTester";

    map.info.height = map_height;
    map.info.width = map_width;
    map.data.resize(map_height * map_width);
    std::fill(map.data.begin(), map.data.end(), 0);

    current_pose.position.x = 200;
    current_pose.position.y = 100;
    current_pose.orientation = tf::createQuaternionMsgFromYaw(pi / 4);

    lane_trajectory.poses.resize(num_samples);
    for (int i = 0; i < lane_trajectory.poses.size(); i++) {
        lane_trajectory.poses[i].pose.position.x = i * map_width / num_samples;
        lane_trajectory.poses[i].pose.position.y = i * map_height / num_samples;
        lane_trajectory.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(pi / 4);
    }

    image = cv::Mat(cv::Size(map_height * scale, map_width * scale), CV_8UC3, cv::Scalar::all(0));
    cv::namedWindow(window_name, 0);
    cv::setMouseCallback(window_name, callbackWrapper, this);
}

RoadNavigationTester::RoadNavigationTester(const RoadNavigationTester& orig) {
}

RoadNavigationTester::~RoadNavigationTester() {
}

int RoadNavigationTester::display() {
    //image = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);

    unsigned char *pixel_ptr = (unsigned char *) image.data;
    int num_channels = image.channels();
    for (int i = 0; i < image.cols; i++) {
        for (int j = 0; j < image.rows; j++) {
            unsigned char value = map.data.at((j / scale) * map_width + i / scale) > 50 ? 255 : 0;
            pixel_ptr[j * image.cols * num_channels + i * num_channels + 0] = value;
            pixel_ptr[j * image.cols * num_channels + i * num_channels + 1] = value;
            pixel_ptr[j * image.cols * num_channels + i * num_channels + 2] = value;
        }
    }

    // TODO: Show orientation
    cv::circle(image,
               cv::Point(current_pose.position.x * scale, current_pose.position.y * scale),
               5, cv::Scalar(255, 0, 0));

    for (int lane_pose_id = 0; lane_pose_id + 1 < lane_trajectory.poses.size(); lane_pose_id++) {
        cv::line(image,
                 cv::Point(lane_trajectory.poses.at(lane_pose_id).pose.position.x * scale,
                           lane_trajectory.poses.at(lane_pose_id).pose.position.y * scale),
                 cv::Point(lane_trajectory.poses.at(lane_pose_id + 1).pose.position.x * scale,
                           lane_trajectory.poses.at(lane_pose_id + 1).pose.position.y * scale),
                 cv::Scalar(0, 0, 255), 3, CV_AA, 0);
    }

    for (int path_pose_id = 0; path_pose_id + 1 < path.poses.size(); path_pose_id++) {
        cv::line(image,
                 cv::Point(path.poses.at(path_pose_id).pose.position.x * scale,
                           path.poses.at(path_pose_id).pose.position.y * scale),
                 cv::Point(path.poses.at(path_pose_id + 1).pose.position.x * scale,
                           path.poses.at(path_pose_id + 1).pose.position.y * scale),
                 cv::Scalar(0, 255, 0), 1, CV_AA, 0);
    }

    cv::imshow(window_name, image);
    return cv::waitKey(10);
}

void RoadNavigationTester::moveAlongThePath() {
    if (path.poses.size() != 0) {
        current_pose = path.poses.at(path.poses.size() / 2).pose;
    } else {
        // raise exception!!
    }
}

void RoadNavigationTester::callback(int event, int x, int y, int flags) {
    switch (event) {
        case CV_EVENT_LBUTTONDOWN:
            current_pose.position.x = x / scale;
            current_pose.position.y = y / scale;
            current_pose.orientation = tf::createQuaternionMsgFromYaw(pi / 4);
            break;
        case CV_EVENT_RBUTTONDOWN:
            addObstacle(x, y, 25);
            break;
    }
}

void RoadNavigationTester::addObstacle(int x, int y, int radius) {
    for (int i = -radius; i <= radius; i++) {
        for (int j = -radius; j <= radius; j++) {
            if (i * i + j * j <= radius * radius) {
                map.data.at(((y + j) / scale) * map_width + (x + i) / scale) = 100;
            }
        }
    }
}

void callbackWrapper(int event, int x, int y, int flags, void* params) {
    RoadNavigationTester *tester = (RoadNavigationTester*) params;
    if (tester != NULL) {
        tester->callback(event, x, y, flags);
    }
}

