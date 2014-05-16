#include "dummy_navigator/dummy_navigator.h"

int main(int argc, char* argv[]){

	ros::init(argc,argv,"dummy_navigator");

	ros::NodeHandle nh;

	geometry_msgs::Pose2D target_pose;
	navigation::State target = navigation_space::DummyNavigator::getTargetLocation();

	target_pose.x = target.x();
	target_pose.y = target.y();
	target_pose.theta = target.theta();

	ros::Publisher pub_target_pose = nh.advertise<geometry_msgs::Pose2D>("/target",10);
	ros::Rate loop_rate(10);

	while (ros::ok())
  	{
  		pub_target_pose.publish(target_pose);
  		loop_rate.sleep();
  	}

  	return 0;
}