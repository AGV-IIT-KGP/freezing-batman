/* 
 * File:   Debugger.cpp
 * Author: auro666
 * 
 * Created on 11 January, 2014, 9:39 PM
 */

#include <Debugger.hpp>

Debugger::Debugger() {
}

Debugger::Debugger(ros::NodeHandle node_handle) {
    map_width = 1000;
    map_height = 1000;
    scale = .5;
    window_name = std::string("Path Planning");

    cv::namedWindow(window_name.c_str(), 0);
    image = cv::Mat(cv::Size(map_height * scale, map_width * scale), CV_8UC3,
                    cv::Scalar::all(255));

    maneuver_subscriber = node_handle.subscribe("situational_planner/maneuver",
                                                2, &Debugger::SetManeuver, this);
    path_subscriber = node_handle.subscribe("local_planner/path", 2,
                                            &Debugger::SetPath, this);
}

Debugger::Debugger(const Debugger& orig) {
}

Debugger::~Debugger() {
}

void Debugger::display(int debug_mode) {
    image = cv::Scalar(255, 255, 255);
    for (unsigned int pose_id = 0; pose_id + 1 < maneuver.poses.size(); pose_id++) {
        cv::line(image,
                 cv::Point(maneuver.poses.at(pose_id).pose.position.x * scale,
                           maneuver.poses.at(pose_id).pose.position.y * scale),
                 cv::Point(maneuver.poses.at(pose_id + 1).pose.position.x * scale,
                           maneuver.poses.at(pose_id + 1).pose.position.y * scale),
                 cv::Scalar(0, 0, 255), 3, CV_AA, 0);
    }

    for (unsigned int pose_id = 0; pose_id + 1 < path.poses.size(); pose_id++) {
        cv::line(image,
                 cv::Point(path.poses.at(pose_id).pose.position.x * scale,
                           path.poses.at(pose_id).pose.position.y * scale),
                 cv::Point(path.poses.at(pose_id + 1).pose.position.x * scale,
                           path.poses.at(pose_id + 1).pose.position.y * scale),
                 cv::Scalar(0, 255, 0), 3, CV_AA, 0);
    }

    if (debug_mode == 1) {
        cv::imshow(window_name.c_str(), image);
        cv::waitKey(10);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "local_planner_debugger");
    ros::NodeHandle node_handle;
    Debugger debugger(node_handle);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        int debug_mode;
        node_handle.getParam("/local_planner_debugger/debug_mode", debug_mode);
        debugger.display(debug_mode);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
