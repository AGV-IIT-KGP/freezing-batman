#include <ros/ros.h>
#include "LaneDectector.h"
void interpreter(int argc, char **argv) {
    ros::init(argc, argv, "lane_detector");
    LaneDetection lane_d;
    image_transport::ImageTransport it(lane_node);
  
     image_transport::Subscriber  lane_subscriber = it.subscribe<cv_bridge::CvImage>("sensors/camera", 10, &LaneDetector::markLane,  &lane_d);

 
  image_transport::Publisher  lane_publisher = it.publish<cv_bridge::CvImage>("interpretor/lane", 10);


    ros::Rate loop_rate(1);
    while (ros::ok()) {


        ros::spinOnce();
        loop_rate.sleep();
    }

}

