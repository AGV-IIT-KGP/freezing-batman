#include <iostream>
#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/impl/point_types.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc,char** argv){
ros::init(argc,argv,"publisher");
ros::NodeHandle n;
ros::Publisher pub=n.advertise<PointCloud> ("publisher",100);

ros::Rate loop_rate(10);
while(ros::ok()){
	PointCloud::Ptr msg (new PointCloud);
	msg->header.frame_id="PointCloud publisher";
	msg->height=msg->width=1;
	msg->points.push_back(pcl::PointXYZ(1.0,2.0,3.0));
	msg->header.stamp=ros::Time::now();
	//ROS_INFO("%f %f\n",msg->points[0].x,msg->points[0].y);
	ros::spinOnce();
	loop_rate.sleep();
}
return 0;
}
