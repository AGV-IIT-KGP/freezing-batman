#ifndef STRATEGY_PLANNER
#define STRATEGY_PLANNER

#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <iosfwd>

static const int buffer_size = 10;
static const int loop_rate_hz = 10;

enum Navigators {
    dummy_navigator = 0,
    nose_navigator = 1,
    waypoint_navigator = 2,
    lane_navigator = 3,
};

enum Planners {
    a_star_seed = 0,
    quick_reflex = 1,
};

class Strategy_Planner {
    geometry_msgs::Pose2D dummy_target_, nose_target_, waypoint_target_, lane_target_;
    geometry_msgs::Pose2D final_target;
    bool is_emergency_;
    //    bool has_target_reached;
    std::string emergency_status;
    std::string which_planner_;
    std::string which_navigator_;
    int navigators_, planners_;
    bool nml_flag, is_confident_;

public:
    void plan(bool is_test_mode, int planner, int navigator);
    void setEmergency(std_msgs::String status);
    void setFinalTarget(geometry_msgs::Pose2D set_target_);
    void setWhichPlanner(std::string planner);
    void setWhichNavigator(std::string navigator);
    void setNmlFlag(std_msgs::Bool flag);
    void setDummyTarget(geometry_msgs::Pose2D proposed_dummy_target_);
    void setNoseTarget(geometry_msgs::Pose2D proposed_nose_target_);
    void setWaypointTarget(geometry_msgs::Pose2D proposed_waypoint_target_);
    void setLaneTarget(geometry_msgs::Pose2D proposed_lane_target_);
    void setNavigator(int navigator);
    void setPlanner(int planner);
    void setConfidence(std_msgs::Bool confidence);

    inline geometry_msgs::Pose2D getDummyTarget() const {
        return dummy_target_;
    }

    inline geometry_msgs::Pose2D getNoseTarget() const {
        return nose_target_;
    }

    inline geometry_msgs::Pose2D getWaypointTarget() const {
        return waypoint_target_;
    }

    inline geometry_msgs::Pose2D getLaneTarget() const {
        return lane_target_;
    }

    inline geometry_msgs::Pose2D getFinalTarget() const {
        return final_target;
    }

    inline std_msgs::String getWhichPlanner() const {
        std_msgs::String pub_str;
        pub_str.data = which_planner_;
        return pub_str;
    }

    inline std_msgs::String getWhichNavigator() const {
        std_msgs::String pub_str;
        pub_str.data = which_navigator_;
        return pub_str;
    }
};
#endif