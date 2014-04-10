/* 
 * File:   Debugger.cpp
 * Author: samuel
 * 
 * Created on 7 January, 2014, 1:26 PM
 */

#include <ros/ros.h>
#include <control/Debugger.hpp>

Debugger::Debugger() {
    cte_response = std::string("CTE Response");
    path_tracking = std::string("Path Tracking");

    cte_image.create(600, 2000, CV_8UC3);
    cv::namedWindow(cte_response.c_str(), 0);
    for (int i = 0; i < 2000; i++) {
        cte_plot.push_back(300);
        flat_line.push_back(cv::Point(i, 300));
    }

    path_image.create(600, 800, CV_8UC3);
    cv::namedWindow(path_tracking.c_str(), 0);
}

Debugger::Debugger(const Debugger& orig) {
}

Debugger::~Debugger() {
}

void Debugger::display(int debug_mode) {
    cte_image = cv::Scalar(255, 255, 255);
    for (unsigned int pose_id = 0; pose_id + 1 < cte_plot.size(); pose_id++) {
        cv::line(cte_image,
                 cv::Point(pose_id, cte_plot.at(pose_id)),
                 cv::Point(pose_id, cte_plot.at(pose_id + 1)),
                 cv::Scalar(0, 0, 255), 3, CV_AA, 0);
    }
    for (unsigned int pose_id = 0; pose_id + 1 < flat_line.size(); pose_id++) {
        cv::line(cte_image, flat_line.at(pose_id), flat_line.at(pose_id + 1),
                 cv::Scalar(0, 0, 0), 3, CV_AA, 0);
    }

    if (debug_mode == 3 || debug_mode == 1) {
        cv::imshow(cte_response.c_str(), cte_image);
        cv::waitKey(10);
    }

    path_image = cv::Scalar(255, 255, 255);
    for (unsigned int pose_id = 0; pose_id + 1 < target_path.size(); pose_id++) {
        cv::line(path_image, target_path.at(pose_id),
                 target_path.at(pose_id + 1), cv::Scalar(0, 0, 255), 3, CV_AA,
                 0);
    }
    for (unsigned int pose_id = 0; pose_id + 1 < traversed_path.size(); pose_id++) {
        cv::line(path_image, traversed_path.at(pose_id),
                 traversed_path.at(pose_id + 1), cv::Scalar(0, 255, 0), 3,
                 CV_AA, 0);
    }

    if (debug_mode == 3 || debug_mode == 2) {
        cv::imshow(path_tracking.c_str(), path_image);
        cv::waitKey(10);
    }
}

void Debugger::updateCurrentPath(const geometry_msgs::PoseStamped::ConstPtr& pose_ptr) {
    traversed_path.push_back(cv::Point(10 + pose_ptr->pose.position.x, 
                                       300 - pose_ptr->pose.position.y));
}

void Debugger::updateCTEPlotData(const std_msgs::Float64::ConstPtr& cte_ptr) {
    cte_plot.erase(cte_plot.begin());
    cte_plot.push_back((int) 300 + cte_ptr->data * 100);
}

void Debugger::updateTargetPath(const nav_msgs::Path::ConstPtr& target_path_ptr) {
    target_path.resize(target_path_ptr->poses.size());
    for (unsigned int point_id = 0; point_id < target_path_ptr->poses.size(); point_id++) {
        target_path.at(point_id).x = 10 + target_path_ptr->poses.at(point_id).pose.position.x;
        target_path.at(point_id).y = 300 - target_path_ptr->poses.at(point_id).pose.position.y;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller_debugger");
    ros::NodeHandle node_handle;
    Debugger debugger;

    ros::Subscriber cte_subscriber = node_handle.subscribe("controller/cross_track_error", 2,
                                                           &Debugger::updateCTEPlotData, &debugger);
    ros::Subscriber pose_subscriber = node_handle.subscribe("localization/pose", 2,
                                                            &Debugger::updateCurrentPath, &debugger);
    ros::Subscriber path_subscriber = node_handle.subscribe("local_planner/path", 2,
                                                            &Debugger::updateTargetPath, &debugger);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        int debug_mode = 0;
        node_handle.getParam("/controller_debugger/debug_mode", debug_mode);
        debugger.display(debug_mode);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

