#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Pose2D.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "dummy_tester");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_world_map = it.advertise("data_fuser/map", 10);
//    ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("strategy_planner/target", 10);

    srand((unsigned int) time(NULL));

    int height = 1000, width = 1000;
    int minradius = 20, maxradius = 60;
    int minobs = 2, maxobs = 9;
    int upper_margin_zone = 100;
    int lower_margin_zone = 100;

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        geometry_msgs::Pose2D target_pose;

        target_pose.x = rand() % width;
        target_pose.y = (height - rand() % 50);
        target_pose.theta = (rand() % (360))*(2 * M_PI) / 360;

//        pub_target_pose.publish(target_pose);

        cv::Mat image = cv::Mat(height, width, CV_8UC1, cvScalarAll(0));

        unsigned int numberofobs = (rand() % (maxobs - minobs) + minobs);

        //        for (unsigned int i = 0; i < numberofobs; i++) {
        //            cv::circle(image, cvPoint(rand() % width, 100 + rand() % (height - 200)), minradius + rand() % (maxradius - minradius), cvScalar(255), -1);
        //        }


        cv_bridge::CvImage message;
        message.encoding = sensor_msgs::image_encodings::MONO8;
        message.image = image;
        pub_world_map.publish(message.toImageMsg());

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
