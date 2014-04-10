#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <message_filters/subscriber.h>
#include <vector>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <State/State.hpp>
#include "Navigation/RazorImu.h"
#include "geometry_msgs/Pose.h"
#include <boost/bind.hpp>

#define MAP_MAX 1000
#define LOOP_RATE 10

enum Strategies {
    FollowNose = 0,
    TrackWayPoint = 1,
    HectorSLAM =2,
    LaserTestOnly =3,
    PlannerTestOnly =4 ,
    FusionTestOnly =5,
    IGVCBasic =6
};

namespace navigation_space {

    void truncate(double xt, double yt, int& xtt, int& ytt);

    class FollowNoseStrategy {
    public:
         static void calibrateReferenceHeading(double, int);
         static navigation::State getTargetLocation(double);
         static navigation::State getBotLocation();
    };

    class TrackWayPointStrategy {
    public:
         static navigation::State getTargetLocation(double latitude, double longitude, double heading);
         static navigation::State getBotLocation();
    };

    class IGVCBasicStrategy {
    public:
         static navigation::State getTargetLocation(double latitude, double longitude, double heading);
         static navigation::State getBotLocation();
    };
}