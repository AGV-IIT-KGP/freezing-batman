//
//  local_planner.cpp
//  Debugger
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include <debugger.hpp>

namespace navigation {

    Debugger::Debugger() {
        map_max_cols = 1000;
        map_max_rows = 1000;
        node_handle.getParam("debugger/map_max_rows", map_max_rows);
        node_handle.getParam("debugger/map_max_cols", map_max_cols);

        fusion_map_subscriber = node_handle.subscribe("local_planner/map", 10, &Debugger::updateFusionMap, this);
        target_subscriber = node_handle.subscribe("local_planner/target", 10, &Debugger::updateTargetPose, this);
        path_subscriber = node_handle.subscribe("local_planner/path", 10, &Debugger::updatePath, this);

        cv::namedWindow("FusionMap", CV_WINDOW_FREERATIO);

        bot_pose = navigation::State(map_max_cols / 2, map_max_rows / 10, 90, 0);
        local_map = cv::Mat::zeros(map_max_rows, map_max_cols, CV_8UC1);
    }

    void Debugger::updatePath(const nav_msgs::Path& path_msg) {
        int i;
        Pose current_pose;
        int size = path_msg.poses.size();
        path.resize(size);
        for (i = 0; i < size; i++) {
            current_pose.x = path_msg.poses[i].pose.position.x;
            current_pose.y = path_msg.poses[i].pose.position.y;
            path[i] = current_pose;
        }
    }

    void Debugger::updateFusionMap(const sensor_msgs::ImageConstPtr& fusion_map) {
        //TODO : copy function for occupancy grid
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(fusion_map, sensor_msgs::image_encodings::MONO8);
            local_map = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void Debugger::updateStatus(std_msgs::String status) {
        if (status.data == "NO PATH FOUND" || status.data == "OPEN LIST OVERFLOW" || status.data == "TARGET BEHIND") {
            load_in_planner = true;
        } else {
            load_in_planner = false;
        }
    }

    void Debugger::makeMap() {
        cv::Mat img = local_map;
        cv::circle(img, cvPoint(target_pose.x(), local_map.rows - 1 - target_pose.y()), 5, cvScalar(128), -1);
        cv::line(img, cvPoint(target_pose.x(), local_map.rows - 1 - target_pose.y()), cvPoint(target_pose.x() + 15 * cos((target_pose.theta() * M_PI) / 180), local_map.rows - 1 - target_pose.y() - 15 * sin((target_pose.theta() * M_PI) / 180)), cvScalar(128), 1, 8, 0);
        cv::circle(img, cvPoint(bot_pose.x(), local_map.rows - 1 - bot_pose.y()), 5, cvScalar(128), -1);
        cv::line(img, cvPoint(bot_pose.x(), local_map.rows - 1 - bot_pose.y()), cvPoint(bot_pose.x() + 15 * cos((bot_pose.theta() * M_PI) / 180), local_map.rows - 1 - bot_pose.y() - 15 * sin((bot_pose.theta() * M_PI) / 180)), cvScalar(128), 1, 8, 0);
        for (std::vector<Pose>::iterator poseIt = path.begin(); poseIt != path.end(); ++poseIt) {
            const Pose pos = *poseIt;
            cv::circle(img, cv::Point(pos.x, local_map.rows - pos.y - 1), 3, cv::Scalar(255), -1);
        }
    }

    void Debugger::showPath() {
        cv::imshow("FusionMap", local_map);
        cv::waitKey(10);
    }
}

int main(int argc, char* argv[]) {
    const std::string node_name = "debugger";
    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle node_handle;

    navigation::Debugger debugger;
    debugger.node_handle = node_handle;

    int loop_rate_hz = 10;
    node_handle.getParam("debugger/loop_rate", loop_rate_hz);
    ros::Rate loop_rate(loop_rate_hz);
    while (ros::ok()) {
        ros::spinOnce();
        debugger.makeMap();
        debugger.showPath();
        loop_rate.sleep();
    }
}