/* 
 * File:   Trajectory.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 7:34 PM
 */

#ifndef TRAJECTORY_HPP
#define	TRAJECTORY_HPP


#include <iostream>
#include <vector>
#include <PathSegment.hpp>
#include <State.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
 namespace navigation {

    class Trajectory {
    public:
        Trajectory();
        virtual ~Trajectory();

        virtual std::vector<PathSegment*> drawPath(geometry_msgs::Pose current_pose, geometry_msgs::Pose target_pose)=0;
  
    };
}


#endif	/* TRAJECTORY_HPP */

