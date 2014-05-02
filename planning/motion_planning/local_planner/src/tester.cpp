#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <ctime>
#include <cstdlib>
#include "geometry_msgs/Pose.h"
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_tester");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("interpreter/fusion/world_map", 10);
    ros::Publisher pub_target_pose = it.advertise<geometry_msgs::Pose>("/target_pose",10);
    ros::Publisher pub_bot_pose = it.advertise<geometry_msgs::Pose>("/bot_pose",10);

    srand((unsigned int) time(NULL));

    int height=600,width=600;
    int minradius=10,maxradius=40;
    int minobs=2,maxobs=9;

    ros::Rate loop_rate(10);

    while (nh.ok()) 
    {
      cv::Mat image=cv::Mat(height,width,CV_8UC1,cvScalarAll(0));
      
      unsigned int numberofobs=(rand()%(maxobs-minobs)+minobs);
      
      for (unsigned int i=0;i<numberofobs;i++)
      {
        cv::circle(image, cvPoint(rand()%height,rand()%width), minradius+rand()%(maxradius-minradius), cvScalar(255, -1));
      }
      sensor_msgs::ImagePtr msg;
      msg.data = sensor_msgs::CvBridge::cvToImgMsg(image, "mono8");
      pub.publish(msg);

        geometry_msgs::Pose target_pose;
        target_pose.position.x = rand()%width;
        target_pose.position.y = rand()%height;
        target_pose.position.z = rand()%360;

        pub_target_pose.publish(target_pose);

        geometry_msgs::Pose bot_pose;
        bot_pose.position.x = rand()%width;
        bot_pose.position.y = rand()%height;
        bot_pose.position.z = rand()%360;

        pub_bot_pose.publish(bot_pose);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
 }