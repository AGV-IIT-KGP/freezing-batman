/* 
 * File:   RoadNavigationTester.cpp
 * Author: satya
 * 
 * Created on December 14, 2013, 4:21 PM
 */

#include "RoadNavigationTester.hpp"


namespace navigation {

    
    void selectPath(nav_msgs::Path& path_msg) {
        int n_poses = 1000;
        path_msg.poses.resize(n_poses);
        for (int i = 0; i < path_msg.poses.size(); i++) {
            path_msg.poses[i].pose.position.x = 400;
            path_msg.poses[i].pose.position.y = 400;
            path_msg.poses[i].pose.position.z = PI / 4;
        }
    }

    void generate_pos(geometry_msgs::PoseWithCovarianceStamped& current_pos) {
        current_pos.pose.pose.position.x = 300;
        current_pos.pose.pose.position.y = 300;
        current_pos.pose.pose.position.z = 0;
    }

    void bestPath(const nav_msgs::Path::ConstPtr& path_msg) {
        cv::Mat img = cv::Mat(cv::Size(800, 800), CV_8UC3, cv::Scalar::all(0));

        cv::circle(
                img, 
                cv::Point(
                        path_msg->poses.at(0).pose.position.x, 
                        HEIGHT - path_msg->poses.at(0).pose.position.y), 
                5, 
                cv::Scalar::all(255));
        cv::circle(
                img, 
                cvPoint(
                        path_msg->poses.at(path_msg->poses.size() - 1).pose.position.x, 
                        HEIGHT - path_msg->poses.at(path_msg->poses.size() - 1).pose.position.y), 
                5, 
                cv::Scalar::all(255));
        cv::line(
                img, 
                cvPoint(
                        path_msg->poses.at(0).pose.position.x, 
                        HEIGHT - path_msg->poses.at(0).pose.position.y), 
                cvPoint(
                        path_msg->poses.at(path_msg->poses.size() - 1).pose.position.x, 
                        HEIGHT - path_msg->poses.at(path_msg->poses.size() - 1).pose.position.y), 
                cv::Scalar::all(255));
        for (int i = 0; i < path_msg->poses.size() - 1; i++) {
            cv::line(
                img, 
                cvPoint(
                        path_msg->poses.at(i).pose.position.x, 
                        HEIGHT - path_msg->poses.at(i).pose.position.y), 
                cvPoint(
                        path_msg->poses.at(i + 1).pose.position.x, 
                        HEIGHT - path_msg->poses.at(i + 1).pose.position.y), 
                cv::Scalar::all(255));
        }

        cv::imshow("Path Planned B)", img);
        cv::waitKey(0);
    }
}