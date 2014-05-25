#include <vector>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <cmath>
#include <iosfwd>
#include <ios>
#include <iomanip>
#include <sstream>
#include <stdlib.h>

//All distance units are in meter. Conversion has been taken care of before publishing.

static const int loop_rate_hz = 10;
static const int buffer_size = 10;
static const int altitude_preset = 60;

enum Strategy {
    greedy_selector = 0,
    sequential_selector = 1,
};

class WaypointSelector {
    std::string planner_status_;
    int strategy_;
    unsigned int num_visited_waypoints_;
    int num_of_waypoints_;
    bool inside_no_mans_land_;
    std::ifstream waypoints_;
    sensor_msgs::NavSatFix current_gps_position_;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator current_target_ptr;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> > gps_waypoints_;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator last_waypoint_;

public:
    double proximity_;

    bool readWaypoints(std::ifstream& waypoints, std::vector<std::pair<sensor_msgs::NavSatFix, bool> >& gps_waypoints, int& num_of_waypoints, std::string filename);
    geometry_msgs::Pose2D interpret(sensor_msgs::NavSatFix current, sensor_msgs::NavSatFix target);
    double getMod(geometry_msgs::Pose2D pose);
    void set_current_position(sensor_msgs::NavSatFix subscriber_gps);
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator selectNearestWaypoint();
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator selectNextWaypointInSequence();
    bool reachedCurrentWaypoint(std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator target_ptr);
    void set_planner_status(std_msgs::String status);
    WaypointSelector(std::string file, int strategy);
    sensor_msgs::NavSatFix findTarget();
    bool isInsideNoMansLand();
};