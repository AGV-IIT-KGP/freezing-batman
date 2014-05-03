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
#include "geometry_msgs/Pose.h"

 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_tester");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_world_map = it.advertise("interpreter/fusion/world_map", 10);
    ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose>("/target_pose",10);
    ros::Publisher pub_bot_pose = nh.advertise<geometry_msgs::Pose>("/bot_pose",10);

    srand((unsigned int) time(NULL));

    int height=600,width=600;
    int minradius=20,maxradius=60;
    int minobs=2,maxobs=9;
    int upper_margin_zone=50;
    int lower_margin_zone=50;

    ros::Rate loop_rate(10);

    while (ros::ok()) 
    {
      geometry_msgs::Pose bot_pose;
      bot_pose.position.x = rand()%width;
      bot_pose.position.y = rand()%lower_margin_zone;
      bot_pose.position.z = rand()%360;

      pub_bot_pose.publish(bot_pose);

      geometry_msgs::Pose target_pose;
      target_pose.position.x = rand()%width;
      target_pose.position.y = (height-rand()%upper_margin_zone);
      target_pose.position.z = rand()%360;

      pub_target_pose.publish(target_pose);

      cv::Mat image=cv::Mat(height,width,CV_8UC1,cvScalarAll(0));
      
      unsigned int numberofobs=(rand()%(maxobs-minobs)+minobs);
      
      for (unsigned int i=0;i<numberofobs;i++)
      {
        cv::circle(image, cvPoint(rand()%height,rand()%width), minradius+rand()%(maxradius-minradius), cvScalar(255),-1);
      }


      cv_bridge::CvImage message;
      message.encoding = sensor_msgs::image_encodings::MONO8;
      message.image = image;
      pub_world_map.publish(message.toImageMsg());

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
 }