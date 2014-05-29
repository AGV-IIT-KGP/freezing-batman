#include <waypoint_selector.hpp>

typedef long double precnum_t;

bool WaypointSelector::readWaypoints(std::ifstream& waypoints, std::vector<std::pair<sensor_msgs::NavSatFix, bool> >& gps_waypoints, int& num_of_waypoints, std::string filename) {
    std::pair < sensor_msgs::NavSatFix, bool> target;
    std::string line;
    int count = 0;
    int temp_no_waypoints = -1;
    waypoints.open(filename.c_str(), std::ios::in);
    int i = 0;
    while (waypoints.good()) {
        if (i == 0 && waypoints.eof()) {
            waypoints.close();
            return false;
        }

        if (!waypoints.eof()) {
            while (std::getline(waypoints, line)) {
                if (line[0] != '#') {//checks if there is any commented line
                    std::istringstream iss(line);
                    if (i == 0) {//checks if an integer is given in the beginning
                        int num;
                        iss >> num;
                        temp_no_waypoints = num;
                        i++;
                        continue;
                    }
                    std::string lat, lon;
                    while (count < temp_no_waypoints) {//if waypoints are less than the number written exits
                        while (std::getline(iss, lat, ' ') && std::getline(iss, lon, ' ')) {//checks if both lat and lon are given
                            target.first.latitude = strtod(lat.c_str(), NULL);
                            target.first.longitude = strtod(lon.c_str(), NULL);
                            target.first.altitude = altitude_preset;
                            target.second = false;
                            gps_waypoints.push_back(target);
                            count++;
                        }
                        break;
                    }
                }
            }
            num_of_waypoints = count;
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

    return enu_relative_target_2D;
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
    if (!readWaypoints(waypoints_, gps_waypoints_, num_of_waypoints_, file)) {
        std::cout << "exiting";
        exit(1);
    }
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
                    current_target_ptr = selectNextWaypointInSequence();
                }
                if (!reachedCurrentWaypoint(current_target_ptr)) {
                    return current_target_ptr->first;
                }
            }
            break;
        case greedy_selector:
            current_target_ptr = selectNearestWaypoint();
            if (current_target_ptr == gps_waypoints_.end()) {
                return current_gps_position_;
            }
            if (!reachedCurrentWaypoint(current_target_ptr)) {
                return current_target_ptr->first;
            } else {
                current_target_ptr = selectNearestWaypoint();
                return current_target_ptr->first;
            }
            break;
    }
}

bool WaypointSelector::isInsideNoMansLand() {
    return inside_no_mans_land_;
}
