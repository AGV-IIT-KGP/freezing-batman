/* 
 * File:   waypoint_navigator.cpp
 * Author: Abinash Meher
 * 
 * Created on 18 May, 2014, 7:38 AM
 */

#include <waypoint_navigator.hpp>

typedef long double precnum_t;

void WaypointNavigator::set_current_fix(sensor_msgs::NavSatFix current_fix) {
    current_fix_ = current_fix;
}

void WaypointNavigator::set_target_fix(sensor_msgs::NavSatFix target_fix) {
    target_fix_ = target_fix;
}

void WaypointNavigator::set_heading(std_msgs::Float64 heading) {
    heading_ = heading.data;
}

geometry_msgs::Pose2D WaypointNavigator::interpret() {
//     const precnum_t a = 6378137L; // Semi-major axis of the Earth (meters)
//     const precnum_t b = 6356752.3142L; // Semi-minor axis:
//     const precnum_t ae = acos(b / a); // eccentricity:
//     const precnum_t cos2_ae_earth = cos(ae) * cos(ae); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
//     const precnum_t sin2_ae_earth = sin(ae) * sin(ae); // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

//     precnum_t lon = (precnum_t(current_fix_.longitude) * M_PI / 180);
//     precnum_t lat = (precnum_t(current_fix_.latitude) * M_PI / 180);
//     precnum_t N = a / std::sqrt(1 - sin2_ae_earth * std::pow(sin(lat), 2));

//     geometry_msgs::Pose2D enu_relative_target;
//     enu_relative_target.x = (N + current_fix_.altitude) * cos(lat) * cos(lon);
//     enu_relative_target.y = (N + current_fix_.altitude) * cos(lat) * sin(lon);
//     enu_relative_target.theta = (cos2_ae_earth * N + current_fix_.altitude) * sin(lat);

//     lon = (precnum_t(target_fix_.longitude) * M_PI / 180);
//     lat = (precnum_t(target_fix_.latitude) * M_PI / 180);
//     N = a / std::sqrt(1 - sin2_ae_earth * std::pow(sin(lat), 2));

//     geometry_msgs::Pose2D temp;
//     temp.x = (N + target_fix_.altitude) * cos(lat) * cos(lon);
//     temp.y = (N + target_fix_.altitude) * cos(lat) * sin(lon);
//     temp.theta = (cos2_ae_earth * N + target_fix_.altitude) * sin(lat);

//     const double clat = cos((current_fix_.latitude * M_PI / 180)), slat = sin((current_fix_.latitude * M_PI / 180));
//     const double clon = cos((current_fix_.longitude * M_PI / 180)), slon = sin((current_fix_.longitude * M_PI / 180));

//     temp.x -= enu_relative_target.x;
//     temp.y -= enu_relative_target.y;
//     temp.theta -= enu_relative_target.theta;

//     enu_relative_target.x = -slon * temp.x + clon * temp.y;
//     enu_relative_target.y = -clon * slat * temp.x - slon * slat * temp.y + clat * temp.theta;
//     enu_relative_target.theta = clon * clat * temp.x + slon * clat * temp.y + slat * temp.theta;
    const precnum_t a = 6378137L; // Semi-major axis of the Earth (meters)
    const precnum_t b = 6356752.3142L; // Semi-minor axis:
    const precnum_t ae = acos(b / a); // eccentricity:
    const precnum_t cos2_ae_earth = cos(ae) * cos(ae); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
    const precnum_t sin2_ae_earth = sin(ae) * sin(ae); // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

    precnum_t lon = (precnum_t(current_fix_.longitude) * M_PI / 180);
    precnum_t lat = (precnum_t(current_fix_.latitude) * M_PI / 180);
    precnum_t N = a / std::sqrt(1 - sin2_ae_earth * std::pow(sin(lat), 2));

    geometry_msgs::Pose enu_relative_target;
    enu_relative_target.position.x = (N + current_fix_.altitude) * cos(lat) * cos(lon);
    enu_relative_target.position.y = (N + current_fix_.altitude) * cos(lat) * sin(lon);
    enu_relative_target.position.z = (cos2_ae_earth * N + current_fix_.altitude) * sin(lat);

    lon = (precnum_t(target_fix_.longitude) * M_PI / 180);
    lat = (precnum_t(target_fix_.latitude) * M_PI / 180);
    N = a / std::sqrt(1 - sin2_ae_earth * std::pow(sin(lat), 2));

    geometry_msgs::Pose temp;
    temp.position.x = (N + target_fix_.altitude) * cos(lat) * cos(lon);
    temp.position.y = (N + target_fix_.altitude) * cos(lat) * sin(lon);
    temp.position.z = (cos2_ae_earth * N + target_fix_.altitude) * sin(lat);

    const double clat = cos((current_fix_.latitude * M_PI / 180)), slat = sin((current_fix_.latitude * M_PI / 180));
    const double clon = cos((current_fix_.longitude * M_PI / 180)), slon = sin((current_fix_.longitude * M_PI / 180));

    temp.position.x -= enu_relative_target.position.x;
    temp.position.y -= enu_relative_target.position.y;
    temp.position.z -= enu_relative_target.position.z;

    enu_relative_target.position.x = -slon * temp.position.x + clon * temp.position.y;
    enu_relative_target.position.y = -clon * slat * temp.position.x - slon * slat * temp.position.y + clat * temp.position.z;
    enu_relative_target.position.z = clon * clat * temp.position.x + slon * clat * temp.position.y + slat * temp.position.z;

    geometry_msgs::Pose2D enu_relative_target_2D;
    enu_relative_target_2D.x = enu_relative_target.position.x;
    enu_relative_target_2D.y = enu_relative_target.position.y;

    heading_ *= (M_PI / 180.0);
    double alpha = -heading_;
    geometry_msgs::Pose2D bot_relative_target_ENU;
    bot_relative_target_ENU.x = enu_relative_target_2D.x * cos(alpha) + enu_relative_target_2D.y * sin(alpha);
    bot_relative_target_ENU.y = -enu_relative_target_2D.x * sin(alpha) + enu_relative_target_2D.y * cos(alpha);
    bot_relative_target_ENU.theta = M_PI / 2;

    geometry_msgs::Pose2D bot_relative_target_local;

    bot_relative_target_local.x = bot_relative_target_ENU.y;
    bot_relative_target_local.y = -bot_relative_target_ENU.x;
    bot_relative_target_local.theta = bot_relative_target_ENU.theta;

    return bot_relative_target_local;
}