#ifndef POSITION_FORM_TARGET_HPP
#define	POSITION_FORM_TARGET_HPP

#include <environment/Interpreter.hpp>
#include <stdexcept>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <math.h>
#include <string>

class PosefromTarget : public environment::Interpreter {
public:
    void interpret();
    PosefromTarget(int argc, char* argv[]);
    PosefromTarget(const PosefromTarget& orig);
    virtual ~PosefromTarget();
private:
	int id;
	sensor_msgs::NavSatFix target, current;
	geometry_msgs::PoseStamped posestamped;
	geometry_msgs::Pose pose,temp;
	std::string target_topic, gps_topic, pose_topic, posestamped_topic;
	ros::NodeHandle nh;
	ros::Subscriber gps_sub, target_sub;
	ros::Publisher poseStamped_pub;
	ros::Publisher pose_pub;
	void targetCallback(const sensor_msgs::NavSatFix::ConstPtr msg);
	void gpsback(const sensor_msgs::NavSatFix::ConstPtr msg);
	void publish_pose();
	int buffer;
};

#endif	/* POSITION_FORM_TARGET_HPP */
