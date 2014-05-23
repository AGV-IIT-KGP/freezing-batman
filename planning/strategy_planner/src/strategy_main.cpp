#include <strategy_planner/strategy_planner.hpp>

int main(int argc, char* argv[]) {
    Strategy_Planner strategy_planner;

    int planner,navigator;
    bool is_test_mode;
    ros::init(argc, argv, "strategy_planner");
    ros::NodeHandle nh;

    ros::Subscriber sub_local_planner_status = nh.subscribe("/local_planner/status", buffer_size, &Strategy_Planner::setEmergency, &strategy_planner);
    ros::Subscriber sub_dummy_navigator_proposed_target = nh.subscribe("/dummy_navigator/proposed_target", buffer_size, &Strategy_Planner::setDummyTarget, &strategy_planner);
    ros::Subscriber sub_nose_navigator_proposed_target = nh.subscribe("/nose_navigator/proposed_target", buffer_size, &Strategy_Planner::setNoseTarget, &strategy_planner);
    ros::Subscriber sub_waypoint_navigator_proposed_target = nh.subscribe("/waypoint_navigator/proposed_target", buffer_size, &Strategy_Planner::setWaypointTarget, &strategy_planner);
    ros::Subscriber sub_lane_navigator_proposed_target = nh.subscribe("/lane_navigator/proposed_target", buffer_size, &Strategy_Planner::setLaneTarget, &strategy_planner);
    ros::Subscriber sub_nml_flag = nh.subscribe("/waypoint_selector/nml_flag", buffer_size, &Strategy_Planner::setNmlFlag, &strategy_planner);
    ros::Subscriber sub_confidence = nh.subscribe("/lane_detector/confidence", buffer_size, &Strategy_Planner::setConfidence, &strategy_planner);

    ros::Publisher pub_target = nh.advertise<geometry_msgs::Pose2D>("/strategy_planner/target", buffer_size);
    ros::Publisher pub_strategy = nh.advertise<std_msgs::String>("/strategy_planner/which_planner", buffer_size);
    //ros::Publisher pub_target_reached = nh.advertise<std_msgs::Bool>("/strategy_planner/waypoint_arrival", buffer_size);
    ros::Publisher pub_navigator = nh.advertise<std_msgs::String>("/strategy_planner/which_navigator", buffer_size);

    ros::Rate loop_rate(loop_rate_hz);
    while (ros::ok()) {
        ros::spinOnce();
        nh.getParam("/strategy_planner/test_mode",is_test_mode);
        nh.getParam("/strategy_planner/planner",planner);
        nh.getParam("/strategy_planner/navigator",navigator);
        strategy_planner.plan(is_test_mode,planner,navigator);
        pub_target.publish(strategy_planner.getFinalTarget());
        pub_strategy.publish(strategy_planner.getWhichPlanner());
        pub_navigator.publish(strategy_planner.getWhichNavigator());
        //        pub_target_reached.publish(strategy_planner.hasTargetReached());
        loop_rate.sleep();
    }
}