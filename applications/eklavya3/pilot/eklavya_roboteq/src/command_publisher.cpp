#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <time.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "eklavya_roboteq");

  ros::NodeHandle nh;
  
  ros::Publisher vel_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
  
  geometry_msgs::Twist cmdvel;
  double scale = 100;
  double w = 0.55000000;

  ros::Rate loop_rate(2);

  int left_vel = 0;
  int right_vel = 0;
    
  while(ros::ok()) {
    cmdvel.linear.x = (left_vel + right_vel) / (2 * scale); cmdvel.linear.y=0;  cmdvel.linear.z=0;
    cmdvel.angular.x=0; cmdvel.angular.y=0; cmdvel.angular.z = (left_vel - right_vel) / (w * scale);
    vel_pub.publish(cmdvel);
    
    loop_rate.sleep();
    
    left_vel = 50;
    right_vel = 50;
  }
}
