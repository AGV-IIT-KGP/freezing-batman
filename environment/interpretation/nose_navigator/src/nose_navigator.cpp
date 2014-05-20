#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include "RazorImu.h"

int iterations;
double reference_heading;
int MAP_MAX = 1000;

ros::Subscriber sub_imu_sparkFun;
ros::Publisher pub_target;

geometry_msgs::Pose2D getTargetLocation(double heading)
{
	
	double alpha;
	int x, y, z;
    if (heading * reference_heading > 0) {
        alpha = reference_heading - heading;
    } else {
        if (heading - reference_heading > 180) {
            alpha = 360 + reference_heading - heading;
        } else if (reference_heading - heading > 180) {
            alpha = reference_heading - heading - 360;
        } else {
            alpha = reference_heading - heading;
        }
    }

    alpha *= M_PI / 180.0;

    double map_height = 0.875 * MAP_MAX;
    double beta = atan(0.4 * (double)MAP_MAX / (double) map_height);
    double gamma = 3.14 - atan(0.4 * (double)MAP_MAX/ (double)(MAP_MAX - map_height));

    if ((-beta <= alpha) && (alpha <= beta)) {
        x = map_height * tan(alpha) + 0.5 * MAP_MAX;
        y = map_height + 0.1 * MAP_MAX;
    } else if (alpha > beta && alpha < gamma) {
        x = 0.9 * MAP_MAX;
        y = 0.4 * MAP_MAX / tan(alpha) + 0.1 * MAP_MAX;
    } else if (alpha < -beta && alpha > -gamma) {
        x = 0.1 * MAP_MAX;
        y = 0.1 * MAP_MAX - 0.4 * MAP_MAX / tan(alpha);
    } else {
        x = 0.5 * MAP_MAX;
        y = 0.1 * MAP_MAX;
    }
    z = 0;
    
    x = x < 100 ? 100 : x;
    y = y < 100 ? 100 : y;

    geometry_msgs::Pose2D target_pos;
    target_pos.x = x;
    target_pos.y = y;
    target_pos.theta = z;

    return target_pos;

}


void pose_update(const eklavya_imu_sparkfun::RazorImu::ConstPtr message)
{
	geometry_msgs::Pose2D target_pos;
	
	double heading = message->yaw;

	if(iterations<5)
		{
			reference_heading = heading;
			return;
		}

	target_pos = getTargetLocation(heading);
	pub_target.publish(target_pos);

	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nose_navigator");
	ros::NodeHandle nh;
	sub_imu_sparkFun = nh.subscribe("sparkfun_ahra/imuRaw", 10, pose_update);
	pub_target = nh.advertise<geometry_msgs::Pose2D>("/nose_navigator/target", 10);
	

	geometry_msgs::Pose2D target;
	ros::Rate loop_rate(10);
	ROS_INFO("Nose Navigation Pub-Sub started \n");

	iterations = 0;

	while(ros::ok()){
		iterations++;
		// ROS_INFO("I was here \n");
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("Navigation Exited");

	return 0;

}