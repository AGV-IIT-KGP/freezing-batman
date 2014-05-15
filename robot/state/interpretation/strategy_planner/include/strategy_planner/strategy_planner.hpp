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

static const int BUFFER_SIZE = 10;
static const int LOOP_RATE = 10;

enum Navigators {
    Dummy_Navigator = 0,
    Nose_Navigator = 1,
    Waypoint_Navigator = 2,
    Lane_Navigator = 3,
};

enum Planners {
    A_Star_Seed = 0,
    Quick_Response = 1,
};

class Strategy_Planner {
    geometry_msgs::Pose2D dummy_target_, nose_target_, waypoint_target_, lane_target_;
    geometry_msgs::Pose2D final_target;
    bool is_high_priority;
    bool has_target_reached;
    std::string high_priority_status;
    std::string which_planner_;
    int Navigators, Planners;

public:
    void strategise();
    void setHighPriority(std_msgs::String status);
    void setFinalTarget(geometry_msgs::Pose2D set_target_);
    void setWhichPlanner(std::string planner_);

    void setDummyTarget(geometry_msgs::Pose2D proposed_dummy_target_);
    void setNoseTarget(geometry_msgs::Pose2D proposed_nose_target_);
    void setWaypointTarget(geometry_msgs::Pose2D proposed_waypoint_target_);
    void setLaneTarget(geometry_msgs::Pose2D proposed_lane_target_);
    void setNavigator(int navigator_);
    void setPlanner(int planner_);
    void checkifTargetReached();
    std_msgs::Bool hasTargetReached();

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
};
#endif