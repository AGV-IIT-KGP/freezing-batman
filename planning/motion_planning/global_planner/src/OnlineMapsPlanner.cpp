/* 
 * File:   OnlineMapsPlanner.cpp
 * Author: samuel
 * 
 * Created on 22 January, 2014, 12:02 PM
 */

#include <OnlineMapsPlanner.hpp>
#include <fstream>
#include <ros/ros.h>

OnlineMapsPlanner::OnlineMapsPlanner() {
}

OnlineMapsPlanner::OnlineMapsPlanner(std::string points_file_name) {
    initialize(points_file_name);
}

OnlineMapsPlanner::OnlineMapsPlanner(const OnlineMapsPlanner& orig) {
}

OnlineMapsPlanner::~OnlineMapsPlanner() {
}

void OnlineMapsPlanner::extractLatLng(std::string line, double& lat, double& lng) {
    std::istringstream iss(line.c_str());
    char c;
    iss >> lat >> c >> lng;

    if (c != ',') {
        // We have a problem
    }
}

int OnlineMapsPlanner::extractLineCount(std::string line) {
    std::istringstream iss(line.c_str());
    int num_points = 0;
    iss >> num_points;
    return num_points;
}

void OnlineMapsPlanner::initialize(std::string points_file_name) {
    std::string line;
    std::ifstream points_file(points_file_name.c_str());

    if (points_file.is_open()) {
        getline(points_file, line);
        waypoints.poses.resize(extractLineCount(line));
        for (unsigned int i = 0; i < waypoints.poses.size(); i++) {
            double lat, lng;
            getline(points_file, line);
            extractLatLng(line, lat, lng);
            waypoints.poses.at(i).pose.position.x = lat; // Meters
            waypoints.poses.at(i).pose.position.y = lng;
        }
    }
    waypoints.header.seq = 0;
}