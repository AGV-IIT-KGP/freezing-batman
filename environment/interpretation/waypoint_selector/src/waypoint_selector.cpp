#include <waypoint_selector.hpp>

typedef long double precnum_t;

bool WaypointSelector::readWaypoints(std::ifstream& waypoints, std::vector<std::pair<sensor_msgs::NavSatFix, bool> >& gps_waypoints, std::string filename) {
    std::pair < sensor_msgs::NavSatFix, bool> target;
    waypoints.open(filename.c_str(), std::ios::in); //use getParam
    int i = 0;
    while (waypoints.good()) {
        if (i == 0 && waypoints.eof()) {
            waypoints.close();
            return false;
        }

        if (!waypoints.eof()) {
            waypoints >> target.first.latitude;
            waypoints >> target.first.longitude;
            target.second = false;
            gps_waypoints.push_back(target);
            i++;
        } else {
            return true;
        }
    }
    return false;
}

geometry_msgs::Pose2D WaypointSelector::interpret(sensor_msgs::NavSatFix current_fix_, sensor_msgs::NavSatFix target_fix_) {
    const precnum_t a = 6378137L; // Semi-major axis of the Earth (meters)
    const precnum_t b = 6356752.3142L; // Semi-minor axis:
    const precnum_t ae = acos(b / a); // eccentricity:
    const precnum_t cos2_ae_earth = cos(ae) * cos(ae); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
    const precnum_t sin2_ae_earth = sin(ae) * sin(ae); // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

    precnum_t lon = (precnum_t(current_fix_.longitude) * M_PI / 180);
    precnum_t lat = (precnum_t(current_fix_.latitude) * M_PI / 180);
    precnum_t N = a / std::sqrt(1 - sin2_ae_earth * std::pow(sin(lat), 2));

    geometry_msgs::Pose2D enu_relative_target;
    enu_relative_target.x = (N + current_fix_.altitude) * cos(lat) * cos(lon);
    enu_relative_target.y = (N + current_fix_.altitude) * cos(lat) * sin(lon);
    enu_relative_target.theta = (cos2_ae_earth * N + current_fix_.altitude) * sin(lat);

    lon = (precnum_t(target_fix_.longitude) * M_PI / 180);
    lat = (precnum_t(target_fix_.latitude) * M_PI / 180);
    N = a / std::sqrt(1 - sin2_ae_earth * std::pow(sin(lat), 2));

    geometry_msgs::Pose2D temp;
    temp.x = (N + target_fix_.altitude) * cos(lat) * cos(lon);
    temp.y = (N + target_fix_.altitude) * cos(lat) * sin(lon);
    temp.theta = (cos2_ae_earth * N + target_fix_.altitude) * sin(lat);

    const double clat = cos((current_fix_.latitude * M_PI / 180)), slat = sin((current_fix_.latitude * M_PI / 180));
    const double clon = cos((current_fix_.longitude * M_PI / 180)), slon = sin((current_fix_.longitude * M_PI / 180));

    temp.x -= enu_relative_target.x;
    temp.y -= enu_relative_target.y;
    temp.theta -= enu_relative_target.theta;

    enu_relative_target.x = -slon * temp.x + clon * temp.y;
    enu_relative_target.y = -clon * slat * temp.x - slon * slat * temp.y + clat * temp.theta;
    enu_relative_target.theta = clon * clat * temp.x + slon * clat * temp.y + slat * temp.theta;

    return enu_relative_target;
}

double WaypointSelector::getMod(geometry_msgs::Pose2D pose) {
    return std::sqrt(pose.x * pose.x + pose.y * pose.y);
}

void WaypointSelector::set_current_position(sensor_msgs::NavSatFix subscribed_fix) {
    current_gps_position_ = subscribed_fix;
}

std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator WaypointSelector::selectNearestWaypoint() {
    int flagged_index = -1;
    double min_distance;
    for (std::vector < std::pair < sensor_msgs::NavSatFix, bool> >::iterator it = gps_waypoints_.begin(); it != gps_waypoints_.end(); ++it) {
        if (!it->second) {
            if (flagged_index == -1) {
                flagged_index = it - gps_waypoints_.begin();
                min_distance = getMod(interpret(current_gps_position_, it->first));
            }
            if (getMod(interpret(current_gps_position_, it->first)) < min_distance) {
                flagged_index = it - gps_waypoints_.begin();
                min_distance = getMod(interpret(current_gps_position_, it->first));
            }
        }
    }
    if (flagged_index == -1) {
        return gps_waypoints_.end();
    }

    return gps_waypoints_.begin() + flagged_index;
}

std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator WaypointSelector::selectNextWaypointInSequence() {
    int flagged_index = -1;
    if (last_waypoint_ == gps_waypoints_.end()) {
        return gps_waypoints_.begin();
    }

    unsigned int index = (last_waypoint_ - gps_waypoints_.begin() + 1) % gps_waypoints_.size();
    for (; index != (last_waypoint_ - gps_waypoints_.begin()); index = (index + 1) % gps_waypoints_.size()) {
        if (!gps_waypoints_.at(index).second) {
            flagged_index = index;
            break;
        }
    }
    if (flagged_index == -1) {
        if (!gps_waypoints_.at(index).second) {
            flagged_index = index;
        }
    }
    if (flagged_index == -1) {
        return gps_waypoints_.end();
    }

    return gps_waypoints_.begin() + flagged_index;
}

bool WaypointSelector::reachedCurrentWaypoint(std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator target_ptr) {
    double error = getMod(interpret(current_gps_position_, target_ptr->first));
    std::cout << "Error: " << error << std::endl;
    
    if (error < proximity_ || planner_status_ == "TARGET REACHED") {
        target_ptr->second = true;
        last_waypoint_ = target_ptr;
        num_visited_waypoints_++;
        if (num_visited_waypoints_ == gps_waypoints_.size()) {
            inside_no_mans_land_ = false;
        } else {
            inside_no_mans_land_ = true;
        }
        return true;
    }
    return false;
}

void WaypointSelector::set_planner_status(std_msgs::String status) {
    planner_status_ = status.data;
}

WaypointSelector::WaypointSelector(std::string file, int strategy) {
    readWaypoints(waypoints_, gps_waypoints_, file);
    strategy_ = strategy;
    last_waypoint_ = gps_waypoints_.end();
    inside_no_mans_land_ = false;
    num_visited_waypoints_ = 0;
}

sensor_msgs::NavSatFix WaypointSelector::findTarget() {
    switch (strategy_) {
        case sequential_selector:
            if (num_visited_waypoints_ == 0) {
                current_target_ptr = selectNearestWaypoint();
                if (!reachedCurrentWaypoint(current_target_ptr)) {
                    return current_target_ptr->first;
                }
            } else {
                current_target_ptr = selectNextWaypointInSequence();
                if (current_target_ptr == gps_waypoints_.end()) {
                    for (std::vector < std::pair < sensor_msgs::NavSatFix, bool> >::iterator it = gps_waypoints_.begin(); it != gps_waypoints_.end(); it++) {
                        it->second = false;
                    }
                }
                if (!reachedCurrentWaypoint(current_target_ptr)) {
                    return current_target_ptr->first;
                }
            }
            break;
        case greedy_selector:
            current_target_ptr = selectNearestWaypoint();
            if (current_target_ptr == gps_waypoints_.end())return current_gps_position_;
            if (!reachedCurrentWaypoint(current_target_ptr)) {
                return current_target_ptr->first;
            } else {
                current_target_ptr = selectNearestWaypoint();
                return current_target_ptr->first;
            }
            break;
    };
}

bool WaypointSelector::isInsideNoMansLand() {
    return inside_no_mans_land_;
}
