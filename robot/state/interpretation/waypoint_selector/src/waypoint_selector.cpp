#include <waypoint_selector/waypoint_selector.hpp>

typedef long double precnum_t;

bool Waypoint_Selector::readWaypoints(std::ifstream& waypoints, std::vector<std::pair<sensor_msgs::NavSatFix, bool> >& gps_waypoints, std::string filename) {
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

geometry_msgs::Pose2D Waypoint_Selector::interpret(sensor_msgs::NavSatFix current, sensor_msgs::NavSatFix target) {
    geometry_msgs::Pose pose, temp;
    geometry_msgs::Pose2D target_relative_pose;

    static const precnum_t a = 6378137L; // Semi-major axis of the Earth (meters)
    static const precnum_t b = 6356752.3142L; // Semi-minor axis:
    static const precnum_t ae = acos(b / a); // eccentricity:
    static const precnum_t cos2_ae_earth = (cos(ae))*(cos(ae)); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
    static const precnum_t sin2_ae_earth = (sin(ae))*(sin(ae)); // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

    precnum_t lon = (precnum_t(current.longitude) * M_PI / 180);
    precnum_t lat = (precnum_t(current.latitude) * M_PI / 180);
    precnum_t N = a / std::sqrt(1 - sin2_ae_earth * (sin(lat))*(sin(lat)));

    pose.position.x = (N + current.altitude) * cos(lat) * cos(lon);
    pose.position.y = (N + current.altitude) * cos(lat) * sin(lon);
    pose.position.z = (cos2_ae_earth * N + current.altitude) * sin(lat);

    lon = (precnum_t(target.longitude) * M_PI / 180);
    lat = (precnum_t(target.latitude) * M_PI / 180);
    N = a / std::sqrt(1 - sin2_ae_earth * (sin(lat))*(sin(lat)));

    temp.position.x = (N + target.altitude) * cos(lat) * cos(lon);
    temp.position.y = (N + target.altitude) * cos(lat) * sin(lon);
    temp.position.z = (cos2_ae_earth * N + target.altitude) * sin(lat);

    const double clat = cos((current.latitude * M_PI / 180)), slat = sin((current.latitude * M_PI / 180));
    const double clon = cos((current.longitude * M_PI / 180)), slon = sin((current.longitude * M_PI / 180));

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

double Waypoint_Selector::getMod(geometry_msgs::Pose2D pose) {
    return std::sqrt((pose.x * pose.x)+(pose.y)*(pose.y));
}

void Waypoint_Selector::setCurrentPosition(sensor_msgs::NavSatFix subscriber_gps) {
    current_gps_position_ = subscriber_gps;
}

std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator Waypoint_Selector::selectNearestWaypoint() {
    int flag = -1;
    double min_distance;
    for (std::vector < std::pair < sensor_msgs::NavSatFix, bool> >::iterator it = gps_waypoints_.begin(); it != gps_waypoints_.end(); ++it) {
        if (!it->second) {
            if (flag == -1) {
                flag = it - gps_waypoints_.begin();
                min_distance = getMod(interpret(current_gps_position_, it->first));
            }
            if (getMod(interpret(current_gps_position_, it->first)) < min_distance) {
                flag = it - gps_waypoints_.begin();
                min_distance = getMod(interpret(current_gps_position_, it->first));
            }
        }
    }
    if (flag == -1)return gps_waypoints_.end();

    return gps_waypoints_.begin() + flag;
}

std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator Waypoint_Selector::selectNextWaypointInSequence() {
    int flag = -1;
    if (last_waypoint_ == gps_waypoints_.end()) {
        return gps_waypoints_.begin();
    }

    unsigned int index = last_waypoint_ - gps_waypoints_.begin() + 1;
    for (; index != (last_waypoint_ - gps_waypoints_.begin()); index = (index + 1) % gps_waypoints_.size()) {
        if (!gps_waypoints_.at(index).second) {
            flag = index;
            break;
        }
    }
    if (flag == -1) {
        if (!gps_waypoints_.at(index).second) {
            flag = index;
        }
    }
    if (flag == -1)return gps_waypoints_.end();

    return gps_waypoints_.begin() + flag;
}

bool Waypoint_Selector::seeifReached(std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator target_ptr) {
    if (getMod(interpret(current_gps_position_, target_ptr->first)) < proximity || planner_status_ == "TARGET REACHED") {
        target_ptr->second = true;
        last_waypoint_ = target_ptr;
        no_visited_waypoints_++;
        if (no_visited_waypoints_ == gps_waypoints_.size()) is_nml_ = false;
        else is_nml_ = true;
        return true;
    }
    return false;
}

void Waypoint_Selector::setPlannerStatus(std_msgs::String status) {
    planner_status_ = status.data;
}

Waypoint_Selector::Waypoint_Selector(std::string file, int strategy) {
    readWaypoints(waypoints_, gps_waypoints_, file);
    strategy_ = strategy;
    last_waypoint_ = gps_waypoints_.end();
    is_nml_ = false;
    no_visited_waypoints_ = 0;
}

sensor_msgs::NavSatFix Waypoint_Selector::getTarget() {
    switch (strategy_) {
        case sequential_selector:
            if (no_visited_waypoints_ == 0) {
                current_target_ptr = selectNearestWaypoint();
                if (!seeifReached(current_target_ptr)) {
                    return current_target_ptr->first;
                }
            } else {
                current_target_ptr = selectNextWaypointInSequence();
                if (current_target_ptr == gps_waypoints_.end()) {
                    for (std::vector < std::pair < sensor_msgs::NavSatFix, bool> >::iterator it = gps_waypoints_.begin(); it != gps_waypoints_.end(); it++) {
                        it->second = false;
                    }
                }
                if (!seeifReached(current_target_ptr)) {
                    return current_target_ptr->first;
                }
            }
            break;
        case greedy_selector:
            current_target_ptr = selectNearestWaypoint();
            if (current_target_ptr == gps_waypoints_.end())return current_gps_position_;
            if (!seeifReached(current_target_ptr)) {
                return current_target_ptr->first;
            } else {
                current_target_ptr = selectNearestWaypoint();
                return current_target_ptr->first;
            }
            break;
    };
}

bool Waypoint_Selector::ifnml() {
    return is_nml_;
}
