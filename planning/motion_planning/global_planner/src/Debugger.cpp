/* 
 * File:   Debugger.cpp
 * Author: samuel
 * 
 * Created on 24 January, 2014, 2:13 AM
 */

#include <fstream>
#include <Debugger.hpp>

Debugger::Debugger() {
}

Debugger::Debugger(ros::NodeHandle node_handle) {
    initialize();

    cv::namedWindow(window_name.c_str(), 0);
    image = cv::Mat(cv::Size(map_height * scale, map_width * scale), CV_8UC3, cv::Scalar::all(255));

    pose_subscriber = node_handle.subscribe("localization/pose", 2, &Debugger::setCurrent_pose, this);
    waypoint_subscriber = node_handle.subscribe("global_planner/waypoints", 2, &Debugger::setWaypoints, this);
}

Debugger::Debugger(const Debugger& orig) {
}

Debugger::~Debugger() {
}

void Debugger::display(int debug_mode) {
    image = cv::Scalar(255, 255, 255);

    for (unsigned int pose_id = 0; pose_id + 1 < waypoints.poses.size(); pose_id++) {
        cv::line(image,
                 cv::Point(waypoints.poses.at(pose_id).pose.position.x * scale,
                           waypoints.poses.at(pose_id).pose.position.y * scale),
                 cv::Point(waypoints.poses.at(pose_id + 1).pose.position.x * scale,
                           waypoints.poses.at(pose_id + 1).pose.position.y * scale),
                 cv::Scalar(0, 0, 255), 3, CV_AA, 0);
    }

    for (unsigned int pose_id = 0; pose_id < waypoints.poses.size(); pose_id++) {
        cv::circle(image,
                   cv::Point(waypoints.poses.at(pose_id).pose.position.x * scale,
                             waypoints.poses.at(pose_id).pose.position.y * scale),
                   5, cv::Scalar(0, 255, 0), 3, CV_AA, 0);
    }

    cv::circle(image,
               cv::Point(current_pose.position.x * scale,
                         current_pose.position.y * scale),
               5, cv::Scalar(255, 0, 0), 3, CV_AA, 0);

    if (debug_mode == 1) {
        cv::imshow(window_name.c_str(), image);
        cv::waitKey(10);
    }
}

void Debugger::dumpCTEPlot() {
    double scale = 20;
    image = cv::Mat(cv::Size(map_height * scale, map_width * scale), CV_8UC3, cv::Scalar::all(255));

    for (unsigned int pose_id = 0; pose_id + 1 < waypoints.poses.size(); pose_id++) {
        cv::line(image,
                 cv::Point(waypoints.poses.at(pose_id).pose.position.x * scale,
                           waypoints.poses.at(pose_id).pose.position.y * scale),
                 cv::Point(waypoints.poses.at(pose_id + 1).pose.position.x * scale,
                           waypoints.poses.at(pose_id + 1).pose.position.y * scale),
                 cv::Scalar(0, 0, 255), 3, CV_AA, 0);
    }

    for (unsigned int pose_id = 0; pose_id < waypoints.poses.size(); pose_id++) {
        cv::circle(image,
                   cv::Point(waypoints.poses.at(pose_id).pose.position.x * scale,
                             waypoints.poses.at(pose_id).pose.position.y * scale),
                   5, cv::Scalar(0, 255, 0), 3, CV_AA, 0);
    }

    for (unsigned int pose_id = 0; pose_id + 1 < traversed_path.poses.size(); pose_id++) {
        cv::line(image,
                 cv::Point(traversed_path.poses.at(pose_id).pose.position.x * scale,
                           traversed_path.poses.at(pose_id).pose.position.y * scale),
                 cv::Point(traversed_path.poses.at(pose_id + 1).pose.position.x * scale,
                           traversed_path.poses.at(pose_id + 1).pose.position.y * scale),
                 cv::Scalar(255, 0, 0), 3, CV_AA, 0);
    }

    cv::imwrite("../results/traversed_path.jpg", image);
}

void Debugger::initialize() {
    map_width = 100; // Meters
    map_height = 100; // Meters
    scale = 5;
    window_name = std::string("Global Map");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_planner_debugger");
    ros::NodeHandle node_handle;
    Debugger debugger(node_handle);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        int debug_mode;
        node_handle.getParam("/global_planner_debugger/debug_mode", debug_mode);
        debugger.display(debug_mode);
        ros::spinOnce();
        loop_rate.sleep();
    }

    debugger.dumpCTEPlot();

    return 0;
}
