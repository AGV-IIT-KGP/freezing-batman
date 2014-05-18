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
    if (status.data == "NO PATH FOUND" || status.data == "OPEN LIST OVERFLOW") {
        is_emergency_ = false;
    } else {
        is_emergency_ = true;
    }
    emergency_status = (status.data);
}

void Strategy_Planner::setNmlFlag(std_msgs::Bool flag) {
    run_waypoint_navigator = flag.data;
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
    planners = planner; //see enum for int values
}

void Strategy_Planner::setNavigator(int navigator_) {
    navigators = navigator_; //see enum for int values
}

void Strategy_Planner::plan() {
    if (!is_emergency_) {
        setPlanner(0);
    } else {
        setPlanner(1);
    }
    if (!run_waypoint_navigator) {
        setNavigator(3);
    } else {
        setNavigator(2);
    }
    switch (planners) {
        case quick_response:
            setWhichPlanner("Quick_Response");
            break;
        default:
            setWhichPlanner("A_Star_Seed");
    }
    switch (navigators) {
        case lane_navigator:
            setWhichNavigator("Lane_Navigator");
            break;
        default:
            setWhichNavigator("Waypoint_Navigator");
    }

}
