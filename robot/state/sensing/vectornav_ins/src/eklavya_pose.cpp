#include <iostream>
#include <fstream>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "vectornav.h"
#include "serial_lnx.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <tf/transform_datatypes.h>
#include <mrpt/topography.h>
#include <mrpt/topography/data_types.h>

#include <robot/Sensor.hpp>
#include <ros/ros.h>

#define PI 3.141
#define ERROR_THRESHOLD 2.5
#define _GPS_PUBLISHER_BUFFER 10
#define _IMU_PUBLISHER_BUFFER 10
#define _POSE_PUBLISHER_BUFFER 10

/* Change the connection settings to your configuration. */
const char* const COM_PORT = "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FTUTUVO5-if00-port0";
const int BAUD_RATE = 115200;

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;
typedef struct ll{
    double latitude;
    double longitude;
}latlong;

int current_target=0;
int NO_OF_TARGETS=0;

void nextTarget(const std_msgs::String::ConstPtr& msg){
    current_target = (current_target++)%NO_OF_TARGETS;
}

class vectornav_imu : public robot::

int main(int argc, char** argv) {
    ros::init(argc, argv, "Position_data");
    ros::NodeHandle n;
    std::ifstream file; // File reading variable
    file.open("../src/targets.txt");
    latlong *targets;
    double error=0;
    file>>NO_OF_TARGETS;
    ros::Subscriber sub = n.subscribe("target_reached", 100, nextTarget);

    targets=(latlong*)malloc(NO_OF_TARGETS*sizeof(latlong));

    for(int i=0;i<NO_OF_TARGETS;i++)
    {
        file>>targets[i].latitude;
        file>>targets[i].longitude;
    }

    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("fix", _GPS_PUBLISHER_BUFFER);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", _IMU_PUBLISHER_BUFFER);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("pose", _POSE_PUBLISHER_BUFFER);
    ros::Publisher yaw_pub = n.advertise<std_msgs::Float32>("yaw", _POSE_PUBLISHER_BUFFER);
    ros::Publisher yaw_rate_pub = n.advertise<std_msgs::Float32>("yaw_rate", _POSE_PUBLISHER_BUFFER);
    ros::Publisher pitch_pub = n.advertise<std_msgs::Float32>("pitch", _POSE_PUBLISHER_BUFFER);
    ros::Publisher roll_pub = n.advertise<std_msgs::Float32>("roll", _POSE_PUBLISHER_BUFFER);
    // Defining the publish message types
    sensor_msgs::NavSatFix _gps;
    sensor_msgs::Imu _imu;
    geometry_msgs::Pose _pose;
    std_msgs::Float32 _yaw;
    std_msgs::Float32 _yaw_rate;
    std_msgs::Float32 _pitch;
    std_msgs::Float32 _roll;
    btQuaternion tf_angles;

    unsigned short gpsWeek, status;
    unsigned char gpsFix, numberOfSatellites;
    float speedAccuracy, timeAccuracy;
    float attitudeUncertainty, positionUncertainty, velocityUncertainty;
    double gpsTime;
    double latitude, longitude, altitude;
    VnVector3 ypr, latitudeLognitudeAltitude, nedVelocity, positionAccuracy;
    Vn200 vn200;

    vn200_connect(&vn200, COM_PORT, BAUD_RATE);

    ROS_INFO("Publishing Data");

    int ctr = 0;

    while (ros::ok()) { 
        vn200_getGpsSolution(
                &vn200,
                &gpsTime,
                &gpsWeek,
                &gpsFix,
                &numberOfSatellites,
                &latitudeLognitudeAltitude,
                &nedVelocity,
                &positionAccuracy,
                &speedAccuracy,
                &timeAccuracy);
        ROS_INFO("[%lf][%lf] from %d satellites,{%lf}",targets[current_target].latitude,targets[current_target].longitude,numberOfSatellites,error);
        
        vn200_getInsSolution(
                &vn200,
                &gpsTime,
                &gpsWeek,
                &status,
                &ypr,
                &latitudeLognitudeAltitude,
                &nedVelocity,
                &attitudeUncertainty,
                &positionUncertainty,
                &velocityUncertainty);

        VnVector3 magnetic;
        VnVector3 acceleration;
        VnVector3 angularRate;
        float temperature;
        float pressure;
        vn200_getCalibratedSensorMeasurements(
                &vn200,
                &magnetic,
                &acceleration,
                &angularRate,
                &temperature,
                &pressure);

        //ROS_INFO("Angular Rate: %f", angularRate.c0 * (180 / 3.14));

        latitude = latitudeLognitudeAltitude.c0;
        longitude = latitudeLognitudeAltitude.c1;
        altitude = latitudeLognitudeAltitude.c2;

        /*    int lim = 3;
            if(ctr == lim) {
              p0.lat = latitude;
              p0.lon = longitude;
              p0.height = altitude;
      
              cout << "REFERENCE SET" << endl;
              ctr++;
            } else if (ctr < lim) {
              ctr++;
              sleep(1);
              continue;
            }
         */

        const mrpt::topography::TGeodeticCoords p0(
                latitude, // lat in deg of origin
                longitude, // lon in deg of origin
                altitude);

        mrpt::topography::TGeodeticCoords p1(
                targets[current_target].latitude,
                targets[current_target].longitude,
                /*22.317733,
                87.309101,*/
                /*22.319575000000000000, // lat in deg  of origin
                87.298412000000000000,*/ // lon in deg of origin
                6);

        mrpt::poses::TPoint3D p;

        /// p1 : TARGET
        /// p0 : CURRENT LOCATION OF BOT
        /// p : Location of Target w.r.t the current location of bot in ENU Coord Frame

        mrpt::topography::geodeticToENU_WGS84(p1, p, p0);

        //Assigning the data to publishing topics
        tf_angles.setEuler(ypr.c0, ypr.c1, ypr.c2);

        _yaw.data = ypr.c0;
        _pitch.data = ypr.c1;
        _roll.data = ypr.c2;

        _yaw_rate.data = angularRate.c2;

        /// POSE.POSITION : Position of target w.r.t the bot
        /// POSE.ORIENTATION : Orientation of the bot

        //Pose data
        error=sqrt( p.x*p.x+p.y*p.y);
        if(error<ERROR_THRESHOLD){
//            current_target=(current_target+1)%NO_OF_TARGETS;
            break;
        }
        
        
        ros::spinOnce();
        
        _pose.position.x = p.x;
        _pose.position.y = p.y;
        _pose.position.z = p.z;
        _pose.orientation.x = tf_angles.x();
        _pose.orientation.y = tf_angles.y();
        _pose.orientation.z = tf_angles.z();
        _pose.orientation.w = tf_angles.w();

        //IMU data
        _imu.orientation.x = tf_angles.x();
        _imu.orientation.y = tf_angles.y();
        _imu.orientation.z = tf_angles.z();
        _imu.orientation.w = tf_angles.w();
        _imu.angular_velocity.x = nedVelocity.c0;
        _imu.angular_velocity.y = nedVelocity.c1;
        _imu.angular_velocity.z = nedVelocity.c2;

        //GPS data
        _gps.latitude = latitudeLognitudeAltitude.c0;
        _gps.longitude = latitudeLognitudeAltitude.c1;
        _gps.altitude = latitudeLognitudeAltitude.c2;

        imu_pub.publish(_imu);
        gps_pub.publish(_gps);
        pose_pub.publish(_pose);
        //yaw_pub.publish(_yaw);
        yaw_rate_pub.publish(_yaw_rate);
        pitch_pub.publish(_pitch);
        roll_pub.publish(_roll);
        //sleep(1);
    }

    vn200_disconnect(&vn200);

    return 0;
}
