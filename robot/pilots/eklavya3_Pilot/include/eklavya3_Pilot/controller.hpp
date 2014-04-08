#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <time.h>
#include "global_planner/Seed.h"

#define LOOP_RATE 10
#define PID_MODE 0

extern ros::Publisher pub_control;

void sendCommand(const global_planner::Seed seed);