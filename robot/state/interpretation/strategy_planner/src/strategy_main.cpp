#include <strategy_planner/strategy_planner.hpp>

int main(int argc, char* argv[]) {
    Strategy_Planner Strategy_;

    ros::init(argc, argv, "strategy_planner");
    ros::NodeHandle nh;

    ros::Subscriber sub_local_planner_status = nh.subscribe("/local_planner/status", BUFFER_SIZE, &Strategy_Planner::setHighPriority, &Strategy_);
    ros::Subscriber sub_dummy_navigator_proposed_target = nh.subscribe("/dummy_navigator/proposed_target", BUFFER_SIZE, &Strategy_Planner::setDummyTarget, &Strategy_);
    ros::Subscriber sub_nose_navigator_proposed_target = nh.subscribe("/nose_navigator/proposed_target", BUFFER_SIZE, &Strategy_Planner::setNoseTarget, &Strategy_);
    ros::Subscriber sub_waypoint_navigator_proposed_target = nh.subscribe("/waypoint_navigator/proposed_target", BUFFER_SIZE, &Strategy_Planner::setWaypointTarget, &Strategy_);
    ros::Subscriber sub_lane_navigator_proposed_target = nh.subscribe("/lane_navigator/proposed_target", BUFFER_SIZE, &Strategy_Planner::setLaneTarget, &Strategy_);

    ros::Publisher pub_target = nh.advertise<geometry_msgs::Pose2D>("/strategy_planner/target", BUFFER_SIZE);
    ros::Publisher pub_strategy = nh.advertise<std_msgs::String>("/strategy_planner/which_planner", BUFFER_SIZE);
    ros::Publisher pub_target_reached = nh.advertise<std_msgs::Bool>("/strategy_planner/waypoint_arrival", BUFFER_SIZE);

    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok()) {
        ros::spinOnce();
        Strategy_.strategise();
        pub_target.publish(Strategy_.getFinalTarget());
        pub_strategy.publish(Strategy_.getWhichPlanner());
        pub_target_reached.publish(Strategy_.hasTargetReached());
        loop_rate.sleep();
    }
}