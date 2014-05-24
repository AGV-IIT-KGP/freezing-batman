#include <cmath>
#include <cstdlib>
#include <ctime>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "dummy_tester");
    ros::NodeHandle node_handle;
    
    image_transport::ImageTransport image_transport(node_handle);
    image_transport::Publisher map_publisher = image_transport.advertise("data_fuser/map", 1000);
    ros::Publisher target_publisher = node_handle.advertise<geometry_msgs::Pose2D>("strategy_planner/target", 1000);

    srand((unsigned int) time(NULL));

    int height = 1000, width = 1000;
    int minradius = 20, maxradius = 60;
    int minobs = 2, numberofobs = 6;
    int upper_margin_zone = 100;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        geometry_msgs::Pose2D target_pose;
        target_pose.x = rand() % width;
        target_pose.y = (.5 * height);
        target_pose.theta = (rand() % (360))*(2 * M_PI) / 360;
        target_publisher.publish(target_pose);

        cv::Mat image = cv::Mat(height, width, CV_8UC1, cvScalarAll(0));
        for (unsigned int i = 0; i < numberofobs; i++) {
            cv::circle(image, cvPoint(rand() % width, 100 + rand() % (height - 200)), minradius + rand() % (maxradius - minradius), cvScalar(255), -1);
        }
        cv_bridge::CvImage message;
        message.encoding = sensor_msgs::image_encodings::MONO8;
        message.image = image;
        map_publisher.publish(message.toImageMsg());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
