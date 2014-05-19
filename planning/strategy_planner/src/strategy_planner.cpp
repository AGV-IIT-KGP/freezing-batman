#include <strategy_planner/strategy_planner.hpp>

void Strategy_Planner::setDummyTarget(geometry_msgs::Pose2D proposed_dummy_target_) {
    dummy_target_ = proposed_dummy_target_;
}

void Strategy_Planner::setNoseTarget(geometry_msgs::Pose2D proposed_nose_target_) {
    nose_target_ = proposed_nose_target_;
}

void Strategy_Planner::setWaypointTarget(geometry_msgs::Pose2D proposed_waypoint_target_) {
    waypoint_target_ = proposed_waypoint_target_;
}

void Strategy_Planner::setLaneTarget(geometry_msgs::Pose2D proposed_lane_target_) {
    lane_target_ = proposed_lane_target_;
}

void Strategy_Planner::setEmergency(std_msgs::String status) {
    if (status.data == "NO PATH FOUND" || status.data == "OPEN LIST OVERFLOW" || status.data == "TARGET BEHIND") {
        is_emergency_ = true;
    } else {
        is_emergency_ = false;
    }
    emergency_status = (status.data);
}

void Strategy_Planner::setNmlFlag(std_msgs::Bool flag) {
    nml_flag = flag.data;
}

void Strategy_Planner::setConfidence(std_msgs::Bool confidence) {
    is_confident_ = confidence.data;
}

void Strategy_Planner::setFinalTarget(geometry_msgs::Pose2D set_target_) {
    final_target = set_target_;
}

void Strategy_Planner::setWhichPlanner(std::string planner) {
    which_planner_ = planner;
}

void Strategy_Planner::setWhichNavigator(std::string navigator) {
    which_navigator_ = navigator;
}

void Strategy_Planner::setPlanner(int planner) {
    planners_ = planner;
    switch (planner) {//see enum for int values
        case a_star_seed:
            setWhichPlanner(std::string("A_Star_Seed"));
            break;
        case quick_response:
            setWhichPlanner(std::string("Quick_Response"));
            break;
    }
}

void Strategy_Planner::setNavigator(int navigator) {
    navigators_ = navigator;
    switch (navigator) {//see enum for int values
        case dummy_navigator:
            setWhichNavigator(std::string("Dummy_Navigator"));
            setFinalTarget(getDummyTarget());
            break;
        case nose_navigator:
            setWhichNavigator(std::string("Nose_Navigator"));
            setFinalTarget(getNoseTarget());
            break;
        case waypoint_navigator:
            setWhichNavigator(std::string("Waypoint_Navigator"));
            setFinalTarget(getWaypointTarget());
            break;
        case lane_navigator:
            setWhichNavigator(std::string("Lane_Navigator"));
            setFinalTarget(getLaneTarget());
            break;
    }

}

void Strategy_Planner::plan() {
    if (!is_emergency_) {
        setPlanner(a_star_seed);
    } else {
        setPlanner(quick_response);
    }
    if (nml_flag) {
        setNavigator(waypoint_navigator);
    } else {
        if (is_confident_) {
            setNavigator(lane_navigator);
        } else {
            setNavigator(waypoint_navigator);
        }
    }
}
