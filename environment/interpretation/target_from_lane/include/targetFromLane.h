#ifndef targetFromLane_HPP
#define	targetFromLane_HPP

#include <environment/Interpreter.hpp>
#include <stdexcept>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <math.h>
#include <string>


class targetFromLane : public environment::Interpreter {
public:
    void interpret();
    targetFromLane(int argc, char* argv[]);
    //PosefromTarget(const PosefromTarget& orig);
    virtual ~PosefromTarget();
private:
	int id;
	sensor_msgs::current;
	geometry_msgs::Pose pose;
	std::string lane_topic, gps_topic, pose_topic;
	ros::NodeHandle n;
	ros::Subscriber gps_sub, lane_sub;
	ros::Publisher pose_pub;
	void gpsback(const sensor_msgs::NavSatFix::ConstPtr msg);
};

#endif	/* POSITION_FORM_TARGET_HPP */