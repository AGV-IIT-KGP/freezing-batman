#include "strategy_planner/strategy_planner.hpp"

void strategise()
{
	
}

int main(int argc,char* argv[])
{
	int high_priority_status,low_priority_status;

	ros::init(argc,argv,"strategy_planner");

	ros::NodeHandle nh;

	ros::Subscriber sub_local_planner_status = nh.subscribe("/local_planner/status",BUFFER_SIZE,setHighPriority);

	ros::Subscriber sub_dummy_navigator_proposed_target 	= nh.subscribe("/dummy_navigator/proposed_target",BUFFER_SIZE,setDummyTarget);
	ros::Subscriber sub_nose_navigator_proposed_target 		= nh.subscribe("/nose_navigator/proposed_target",BUFFER_SIZE,setNoseTarget);
	ros::Subscriber sub_waypoint_navigator_proposed_target 	= nh.subscribe("/waypoint_navigator/proposed_target",BUFFER_SIZE,setWaypointTarget);
	ros::Subscriber sub_lane_navigator_proposed_target 		= nh.subscribe("/lane_navigator/proposed_target",BUFFER_SIZE,setLaneTarget);

	ros::Publisher pub_target 	= nh.advertise<geometry_msgs::Pose>("/strategy_planner/target",BUFFER_SIZE);
	ros::Publisher pub_strategy = nh.advertise<std::string>("/strategy_planner/which_planner",BUFFER_SIZE);

	ros::Publisher pub_target_reached = nh.advertise<std::bool>("/strategy_planner/waypoint_arrival",BUFFER_SIZE);

	ros::Rate loop_rate(LOOP_RATE);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
}