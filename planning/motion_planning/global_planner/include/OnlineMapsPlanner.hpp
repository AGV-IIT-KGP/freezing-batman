/* 
 * File:   OnlineMapsPlanner.hpp
 * Author: samuel
 *
 * Created on 22 January, 2014, 12:02 PM
 */

#ifndef ONLINEMAPSPLANNER_HPP
#define	ONLINEMAPSPLANNER_HPP

#include <string>
#include <nav_msgs/Path.h>

class OnlineMapsPlanner {
public:
    OnlineMapsPlanner();
    OnlineMapsPlanner(std::string points_file_name);
    OnlineMapsPlanner(const OnlineMapsPlanner& orig);
    virtual ~OnlineMapsPlanner();

    nav_msgs::Path getWaypoints() {
        waypoints.header.stamp = ros::Time::now();
        waypoints.header.seq += 1;
        return waypoints;
    }

private:
    nav_msgs::Path waypoints;
    
    void extractLatLng(std::string line, double& lat, double& lng);
    int extractLineCount(std::string line);
    void initialize(std::string points_file_name);
};

#endif	/* ONLINEMAPSPLANNER_HPP */

