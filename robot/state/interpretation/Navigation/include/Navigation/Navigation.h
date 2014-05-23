#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/NavSatFix.h>
#include <sparkfun_ahrs/RazorImu.h>
#include <State/State.hpp>

static const int MAP_MAX = 1000;
static const int loop_rate_hz = 10;
static const int PROXIMITY = 200; //in centimeter

enum Strategies {
    FollowNose = 0,
    TrackWaypoint = 1,
    HectorSLAM = 2,
    LaserTestOnly = 3,
    PlannerTestOnly = 4,
    FusionTestOnly = 5,
    IGVCBasic = 6,
    LaneFollowingOnly = 7,
    switch_lane_GPS = 8,
    DummyNavigator = 9,
};

namespace navigation_space {
    void setBotPoseGPS(sensor_msgs::NavSatFix GPS_coordinates);
    void truncate(double xt, double yt, int& xtt, int& ytt);

    class FollowNoseStrategy {
    public:
        static void calibrateReferenceHeading(double, int);
        static navigation::State getTargetLocation(double);
        static navigation::State getBotLocation();
    };

    class DummyNavigator {
    public:
        static navigation::State getBotLocation();
        static navigation::State getTargetLocation();
    };

    class TrackWaypointStrategy {
        static geometry_msgs::Pose target_pose;

    public:
        static std::ifstream *waypoints;
        static int i;
        static void publish(sensor_msgs::NavSatFix);
        static void setTargetPose(geometry_msgs::Pose);
        static void subscribePoseFromTarget();
        static bool readPublishTargetLocation();
        static navigation::State getBotLocation();
        static navigation::State getTargetLocation(double heading);
    };

    class IGVCBasicStrategy {
    public:
        static navigation::State getTargetLocation(double latitude, double longitude, double heading);
        static navigation::State getBotLocation();
    };

    class LaneFollowingStrategy {
    public:
        static navigation::State getTargetLocation(double latitude, double longitude, double heading);
        static navigation::State getBotLocation();
    };
}