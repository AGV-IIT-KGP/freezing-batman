#ifndef VECTORNAV_HPP
#define	VECTORNAV_HPP

#include <environment/Sensor.hpp>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <unistd.h>



#include <vectornav.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>

class vectorNav : public environment::Sensor {
public:
    bool connect();
    bool disconnect();
    bool fetch();
    void publish(int frame_id);

    vectorNav();
    vectorNav(int argc, char** argv);
    vectorNav(const vectorNav& orig);
    virtual ~vectorNav();
private:
	Vn200 vn200;
	Vn100 vn100;
	int BAUD_RATE;
	std::string COM_PORT_vn100;
	std::string COM_PORT_vn200;
	ros::NodeHandle *node_handle;
	std::string node_name;
	std::string gps_topic_name, yaw_topic_name;
	//std::string imu_topic_name, twist_topic_name
	ros::Publisher gps_pub, yaw_pub;
	//ros::Publisher imu_pub, twist_pub;
	sensor_msgs::NavSatFix _gps;
    /*sensor_msgs::Imu _imu;
    geometry_msgs::Twist _twist;*/
    std_msgs::Float64 _yaw;
    btQuaternion tf_angles;
	int message_queue_size;
    void initializeParameters();
    void initializeParameters(int argc, char** argv);
    void setupCommunications();
};

#endif	/* VECTORNAV */

