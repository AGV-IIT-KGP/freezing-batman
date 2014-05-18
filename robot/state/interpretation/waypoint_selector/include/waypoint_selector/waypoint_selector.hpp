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

//All distance units are in meter. Conversion has been taken care of before publishing.

static const int loop_rate_hz = 10;
static const int buffer_size = 10;
static int proximity;

enum Strategy {
    greedy_selector = 0,
    sequential_selector = 1,
};

class Waypoint_Selector {
    std::string planner_status_;
    int strategy_;
    unsigned int no_visited_waypoints_;
    bool is_nml_;
    std::ifstream waypoints_;
    sensor_msgs::NavSatFix current_gps_position_;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator current_target_ptr;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> > gps_waypoints_;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator last_waypoint_;
public:
    bool readWaypoints(std::ifstream& waypoints, std::vector<std::pair<sensor_msgs::NavSatFix, bool> >& gps_waypoints, std::string filename);
    geometry_msgs::Pose2D interpret(sensor_msgs::NavSatFix current, sensor_msgs::NavSatFix target);
    double getMod(geometry_msgs::Pose2D pose);
    void setCurrentPosition(sensor_msgs::NavSatFix subscriber_gps);
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator selectNearestWaypoint();
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator selectNextWaypointInSequence();
    bool seeifReached(std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator target_ptr);
    void setPlannerStatus(std_msgs::String status);
    Waypoint_Selector(std::string file, int strategy);
    sensor_msgs::NavSatFix getTarget();
    bool ifnml();
};