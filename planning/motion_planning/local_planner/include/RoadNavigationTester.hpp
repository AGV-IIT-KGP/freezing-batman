/* 
 * File:   RoadNavigationTester.hpp
 * Author: satya
 *
 * Created on December 14, 2013, 4:21 PM
 */

#ifndef ROADNAVIGATIONTESTER_HPP
#define	ROADNAVIGATIONTESTER_HPP

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#define PI 3.141
#define HEIGHT 800

namespace navigation {

        void bestPath(const nav_msgs::Path::ConstPtr& path_msg);
        void generate_pos(geometry_msgs::PoseWithCovarianceStamped& current_pos);
        void selectPath(nav_msgs::Path& path_msg);

}
#endif	/* ROADNAVIGATIONTESTER_HPP */

