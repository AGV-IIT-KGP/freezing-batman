#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace ros;

void chatterCallback(const std_msgs::String::ConstPtr& msg){
ROS_INFO("I heard [%s]",msg->data.c_str());
}

int main(int argc,char** argv){
init(argc,argv,"subsciber");
NodeHandle n;
Subscriber sub=n.subscribe("publisher",100,chatterCallback);
spin();
return 0;
}
