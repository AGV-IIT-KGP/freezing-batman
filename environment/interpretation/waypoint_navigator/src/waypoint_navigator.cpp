/* 
 * File:   waypoint_navigator.cpp
 * Author: Abinash Meher
 * 
 * Created on 18 May, 2014, 7:38 AM
 */

#include "waypoint_navigator/waypoint_navigator.hpp"

typedef long double precnum_t;

void Waypoint_Navigator::setCurrentGPS(sensor_msgs::NavSatFix current) {
    current_gps_ = current;
}

void Waypoint_Navigator::setTargetGPS(sensor_msgs::NavSatFix target) {
    target_gps_ = target;
}

geometry_msgs::Pose2D Waypoint_Navigator::interpret() {
    geometry_msgs::Pose pose, temp;
    geometry_msgs::Pose2D target_relative_pose;

    static const precnum_t a = 6378137L; // Semi-major axis of the Earth (meters)
    static const precnum_t b = 6356752.3142L; // Semi-minor axis:
    static const precnum_t ae = acos(b / a); // eccentricity:
    static const precnum_t cos2_ae_earth = (cos(ae))*(cos(ae)); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
    static const precnum_t sin2_ae_earth = (sin(ae))*(sin(ae)); // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

    precnum_t lon = (precnum_t(current_gps_.longitude) * M_PI / 180);
    precnum_t lat = (precnum_t(current_gps_.latitude) * M_PI / 180);
    precnum_t N = a / std::sqrt(1 - sin2_ae_earth * (sin(lat))*(sin(lat)));

    pose.position.x = (N + current_gps_.altitude) * cos(lat) * cos(lon);
    pose.position.y = (N + current_gps_.altitude) * cos(lat) * sin(lon);
    pose.position.z = (cos2_ae_earth * N + current_gps_.altitude) * sin(lat);

    lon = (precnum_t(target_gps_.longitude) * M_PI / 180);
    lat = (precnum_t(target_gps_.latitude) * M_PI / 180);
    N = a / std::sqrt(1 - sin2_ae_earth * (sin(lat))*(sin(lat)));

    temp.position.x = (N + target_gps_.altitude) * cos(lat) * cos(lon);
    temp.position.y = (N + target_gps_.altitude) * cos(lat) * sin(lon);
    temp.position.z = (cos2_ae_earth * N + target_gps_.altitude) * sin(lat);

    const double clat = cos((current_gps_.latitude * M_PI / 180)), slat = sin((current_gps_.latitude * M_PI / 180));
    const double clon = cos((current_gps_.longitude * M_PI / 180)), slon = sin((current_gps_.longitude * M_PI / 180));

    temp.position.x -= pose.position.x;
    temp.position.y -= pose.position.y;
    temp.position.z -= pose.position.z;

    pose.position.x = -slon * temp.position.x + clon * temp.position.y;
    pose.position.y = -clon * slat * temp.position.x - slon * slat * temp.position.y + clat * temp.position.z;
    pose.position.z = clon * clat * temp.position.x + slon * clat * temp.position.y + slat * temp.position.z;

    target_relative_pose.x = pose.position.x;
    target_relative_pose.y = pose.position.y;

    return target_relative_pose;
}