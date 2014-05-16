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
        //Subscriber for World Map
        sub_world_map = nh.subscribe("interpreter/fusion/world_map", 10, &Debugger::updateWorldMap, this);
        // topic should same with data published by GPS
        sub_bot_pose = nh.subscribe("/bot_pose", 10, &Debugger::updateBotPose, this);
        // topic published from GPS
        sub_target_pose = nh.subscribe("/target_pose", 10, &Debugger::updateTargetPose, this);

        sub_nav_msgs = nh.subscribe("/path_nav_msgs", 10, &Debugger::updateNavMsg, this);
        sub_status_msg = nh.subscribe("localplanner/status", 10, &Debugger::showStatus, this);

        // it = new image_transport::ImageTransport(nh);
        // pub_path_image = it->advertise("/pathImage", 1); //Publisher for final path image 



        local_map = cv::Mat::zeros(MAP_MAX, MAP_MAX, CV_8UC1);

        ROS_INFO("Local Planner(AStarSeed) started.... ");
        ROS_INFO("Publisher : \"/pathImage\" .... ");
        ROS_INFO("Subscriber : \"interpreter/fusion/world_map\", \"/bot_pose\", \"/target_pose\" .... ");
    }

    void Debugger::showStatus(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Current Status: [%s]", msg->data.c_str());
        printf("%s", msg->data.c_str());
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
        std::cout << "Showing A Path\n";
        for (std::vector<Pose>::iterator poseIt = path.begin(); poseIt != path.end(); ++poseIt) {
            const Pose pos = *poseIt;
            cv::circle(local_map, cv::Point( pos.x, local_map.rows - pos.y - 1), 3, cv::Scalar(255), -1);
            // cv::circle(local_map, cv::Point(my_bot_location.x() + pos.x, local_map.rows - (my_bot_location.y() + pos.y) - 1), 3, cv::Scalar(255), -1);

        }
        
    }

    // void Debugger::publishImage() {
    //     cv_bridge::CvImage out_msg;
    //     out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    //     out_msg.image = local_map;
    //     pub_path_image.publish(out_msg.toImageMsg());
    //     cv::imshow("view", local_map);
    //     cvWaitKey(10);

    // }

    void Debugger::showPath() {
        cv::imshow("view", local_map);
        cvWaitKey(10);
    }



}

int main(int argc, char* argv[])
{
        const std::string node_name = "debugger";
        ros::init(argc, argv, node_name.c_str());
        navigation::Debugger debugger;
        ros::NodeHandle nh;
        debugger.nh = nh;
        ros::Rate loop_rate(LOOP_RATE);
        
        while (ros::ok()) {
            ros::spinOnce();
            debugger.makeMap();
            
            debugger.showPath();
            loop_rate.sleep();
        }

}