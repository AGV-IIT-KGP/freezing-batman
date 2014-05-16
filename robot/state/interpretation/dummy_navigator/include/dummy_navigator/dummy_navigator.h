#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <message_filters/subscriber.h>
#include <vector>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <State/State.hpp>
#include <sensor_msgs/NavSatFix.h>
#include "geometry_msgs/Pose2D.h"
#include <boost/bind.hpp>
#include <cmath>

static const int MAP_MAX=1000;
static const int LOOP_RATE=10;
static const int PROXIMITY=200;//in centimeter

namespace navigation_space {
	class DummyNavigator{
	public:
		static navigation::State getBotLocation();
		static navigation::State getTargetLocation();
	};
}
