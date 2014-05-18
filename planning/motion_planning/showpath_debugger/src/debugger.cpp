//
//  local_planner.cpp
//  Debugger
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include "debugger.hpp"

namespace navigation {

    Debugger::Debugger() {
        int MAP_MAX_COLS, MAP_MAX_ROWS;
        sub_world_map = nh.subscribe("data_fuser/map", 10, &Debugger::updateWorldMap, this);

        sub_target_pose = nh.subscribe("strategy_planner/target", 10, &Debugger::updateTargetPose, this);

        sub_nav_msgs = nh.subscribe("local_planner/path", 10, &Debugger::updateNavMsg, this);
        sub_status_msg = nh.subscribe("local_planner/status", 10, &Debugger::showStatus, this);

        nh.getParam("debugger/map_max_rows", MAP_MAX_ROWS);
        nh.getParam("debugger/map_max_cols", MAP_MAX_COLS);

        my_bot_location = navigation::State(MAP_MAX_COLS/2, MAP_MAX_ROWS/10, 90, 0);
        
        local_map = cv::Mat::zeros(MAP_MAX_ROWS, MAP_MAX_COLS, CV_8UC1);

    }

    void Debugger::showStatus(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Current Status: %s", msg->data.c_str());
    }

    void Debugger::updateNavMsg(const nav_msgs::Path& msg) {
        int i;
        Pose current_pose;
        int size = msg.poses.size();
        path.resize(size);
        for (i = 0; i < size; i++) {
            current_pose.x = msg.poses[i].pose.position.x;
            current_pose.y = msg.poses[i].pose.position.y;
            path[i] = current_pose;
        }
    }

    void Debugger::updateWorldMap(const sensor_msgs::ImageConstPtr& world_map) {
        //TODO : copy function for occupancy grid
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(world_map, sensor_msgs::image_encodings::MONO8);
            local_map = cv_ptr->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void Debugger::makeMap() {
        cv::circle(local_map, cvPoint(my_target_location.x(), local_map.rows - 1 - my_target_location.y()), 5, cvScalar(128), -1);
        cv::line(local_map, cvPoint(my_target_location.x(), local_map.rows - 1 - my_target_location.y()), cvPoint(my_target_location.x() + 15 * cos((my_target_location.theta() * M_PI) / 180), local_map.rows - 1 - my_target_location.y() - 15 * sin((my_target_location.theta() * M_PI) / 180)), cvScalar(128), 1, 8, 0);
        cv::circle(local_map, cvPoint(my_bot_location.x(), local_map.rows - 1 - my_bot_location.y()), 5, cvScalar(128), -1);
        cv::line(local_map, cvPoint(my_bot_location.x(), local_map.rows - 1 - my_bot_location.y()), cvPoint(my_bot_location.x() + 15 * cos((my_bot_location.theta() * M_PI) / 180), local_map.rows - 1 - my_bot_location.y() - 15 * sin((my_bot_location.theta() * M_PI) / 180)), cvScalar(128), 1, 8, 0);
        for (std::vector<Pose>::iterator poseIt = path.begin(); poseIt != path.end(); ++poseIt) {
            const Pose pos = *poseIt;
            cv::circle(local_map, cv::Point(pos.x, local_map.rows - pos.y - 1), 3, cv::Scalar(255), -1);

        }

    }

    void Debugger::showPath() {
        cv::imshow("view", local_map);
        cvWaitKey(10);
    }



}

int main(int argc, char* argv[]) {
    const std::string node_name = "debugger";
    int LOOP_RATE;
    ros::init(argc, argv, node_name.c_str());
    navigation::Debugger debugger;
    ros::NodeHandle nh;
    debugger.nh = nh;
    nh.getParam("debugger/loop_rate", LOOP_RATE);
    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok()) {
        ros::spinOnce();
        debugger.makeMap();
        debugger.showPath();
        loop_rate.sleep();
    }

}