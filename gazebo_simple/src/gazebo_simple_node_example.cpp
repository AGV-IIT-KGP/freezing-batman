#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

ros::Publisher vel_pub_0; 

static geometry_msgs::Twist no_vel;

geometry_msgs::Twist cmd_vel;

void controlBot(geometry_msgs::Twist twist){
  cmd_vel = twist;
  vel_pub_0.publish(cmd_vel);
  cmd_vel = no_vel;
  vel_pub_0.publish(cmd_vel);
  return;
}

/**
 * This tutorial demonstrates simple sending of velocity commands to the IRobot Create in Gazebo.
 */
int main(int argc, char **argv)
{
  no_vel.linear.x = 0;
  no_vel.linear.y = 0;
  no_vel.linear.z = 0;
  no_vel.angular.x = 0;
  no_vel.angular.y = 0;
  no_vel.angular.z = 0;

  ros::init(argc, argv, "CreateController");

  ros::NodeHandle n;   
  ros::Subscriber manual_control = n.subscribe("turtle1/cmd_vel",1,controlBot);

  vel_pub_0 = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);

  ros::Rate loop_rate(5);

  int count = 0;

  while (ros::ok())
  {

    // cmd_vel.linear.x = 0.5;
    // cmd_vel.linear.y = 0;
    // cmd_vel.linear.z = 0;
    // cmd_vel.angular.x = 0;
    // cmd_vel.angular.y = 0;
    // cmd_vel.angular.z = 0.4*sin(count);


    // vel_pub_0.publish(cmd_vel);
    // vel_pub_1.publish(cmd_vel);
    // vel_pub_2.publish(cmd_vel);
    // vel_pub_3.publish(cmd_vel);
    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }

  return 0;
}
