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

void Strategy_Planner::setHighPriority(std_msgs::String status) {
    is_high_priority = true;
    high_priority_status = (status.data);
}

void Strategy_Planner::setFinalTarget(geometry_msgs::Pose2D set_target_) {
    final_target = set_target_;
}

void Strategy_Planner::setWhichPlanner(std::string planner_) {
    which_planner_ = planner_;
}

void Strategy_Planner::setNavigator(int navigator_) {
    Navigators = navigator_; //see enum for int values
}

void Strategy_Planner::setPlanner(int planner_) {
    Planners = planner_; //see enum for int values
}

void Strategy_Planner::checkifTargetReached() {
    if (high_priority_status == std::string("TARGET FOUND")) {
        has_target_reached = true;
    } else has_target_reached = false;
}

std_msgs::Bool Strategy_Planner::hasTargetReached() {
    std_msgs::Bool pub_bool;
    pub_bool.data = has_target_reached;
    return pub_bool;
}

void Strategy_Planner::strategise() {
    checkifTargetReached();
    if (!is_high_priority) {
        switch (Navigators) {
            case Dummy_Navigator:
                Strategy_Planner::setFinalTarget(dummy_target_);
                break;
            case Nose_Navigator:
                Strategy_Planner::setFinalTarget(nose_target_);
                break;
            case Waypoint_Navigator:
                Strategy_Planner::setFinalTarget(waypoint_target_);
                break;
            case Lane_Navigator:
                Strategy_Planner::setFinalTarget(lane_target_);
                break;
        }

        switch (Planners) {
            case A_Star_Seed:
                setWhichPlanner("A_Star_Seed");
                break;
            case Quick_Response:
                setWhichPlanner("Quick_Response");
                break;
        }
    }
}