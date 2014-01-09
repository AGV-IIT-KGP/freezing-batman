/* 
 * File:   RoadNavigationTester.hpp
 * Author: samuel
 *
 * Created on 25 December, 2013, 8:20 PM
 */

#ifndef ROADNAVIGATIONTESTER_HPP
#define	ROADNAVIGATIONTESTER_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class RoadNavigationTester {
public:
    RoadNavigationTester();
    RoadNavigationTester(const RoadNavigationTester& orig);
    virtual ~RoadNavigationTester();

    geometry_msgs::Pose GetCurrent_pose() const {
        return current_pose;
    }

    nav_msgs::Path GetLane_trajectory() const {
        return lane_trajectory;
    }

    nav_msgs::OccupancyGrid GetMap() const {
        return map;
    }

    void SetPath(const nav_msgs::Path::ConstPtr path) {
        this->path = *path;
        this->moveAlongThePath();
    }

    void callback(int event, int x, int y, int flags);
    int display();
    void moveAlongThePath();

private:
    int map_width;
    int map_height;
    int num_samples;
    double scale;
    double pi;
    std::string window_name;
    nav_msgs::OccupancyGrid map;
    geometry_msgs::Pose current_pose;
    nav_msgs::Path lane_trajectory;
    nav_msgs::Path path;
    cv::Mat image;

    void addObstacle(int x, int y, int radius);
};

void callbackWrapper(int event, int x, int y, int flags, void* params);

#endif	/* ROADNAVIGATIONTESTER_HPP */

