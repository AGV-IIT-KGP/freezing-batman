#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
using namespace ros;

int main(int argc,char** argv){
init(argc,argv,"publisher");
NodeHandle n;
Publisher pub=n.advertise<std_msgs::String>("publisher",100);
Rate loop_rate(10);
int count=0;
while(ok()){
std_msgs::String msg;
std::stringstream ss;
ss<< "Hello "<<count<<std::endl;
msg.data=ss.str();
ROS_INFO("%s",msg.data.c_str());

pub.publish(msg);
spinOnce();
loop_rate.sleep();
count++;
}
return 0;}
