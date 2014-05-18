#include "Navigation/Navigation.h"


navigation::State pose;

ros::Publisher pub_target_pose;
ros::Publisher pub_bot_pose;

int iterations, strategy;

void followWaypoints(navigation::State bot_location, navigation::State current_target){
	
	geometry_msgs::Pose target_pose;
	target_pose.position.x = current_target.x();
	target_pose.position.y = current_target.y();
	target_pose.position.z = current_target.theta();

	pub_target_pose.publish(target_pose);

	bot_location = navigation_space::TrackWaypointStrategy::getBotLocation();

	geometry_msgs::Pose bot_pose;
	bot_pose.position.x = bot_location.x();
	bot_pose.position.y = bot_location.y();
	bot_pose.position.z = bot_location.theta();

	pub_bot_pose.publish(bot_pose);
}

void followLane(){

}

double dist(navigation::State a, navigation::State b){
	return sqrt((a.x()-b.x())*(a.x()-b.x())+(a.y()-b.y())*(a.y()-b.y()));
}


void pose_update(const eklavya_imu_sparkfun::RazorImu::ConstPtr message){
	
	// ROS_INFO("strategy : %d iterations : %d \n", strategy, iterations);

	double heading;
	navigation::State current_target, bot_location;

	switch(strategy){
			case IGVCBasic:
			{
				// Initialize heading with yaw value (Check how)
				// Might need multiple subscribers with synchronization
				// current_target = 
				break;
			}

			case FollowNose:
			{
				heading = message->yaw;
				navigation_space::FollowNoseStrategy::calibrateReferenceHeading(heading, iterations);

				if(iterations < 5)
					break;

				current_target = navigation_space::FollowNoseStrategy::getTargetLocation(heading);
				
				geometry_msgs::Pose target_pose;
				target_pose.position.x = current_target.x();
				target_pose.position.y = current_target.y();
				target_pose.position.z = current_target.theta();

				pub_target_pose.publish(target_pose);

				bot_location = navigation_space::FollowNoseStrategy::getBotLocation();

				geometry_msgs::Pose bot_pose;
				bot_pose.position.x = bot_location.x();
				bot_pose.position.y = bot_location.y();
				bot_pose.position.z = bot_location.theta();

				pub_bot_pose.publish(bot_pose);

				break;
			}

			case DummyNavigator:
			{
				current_target = navigation_space::DummyNavigator::getTargetLocation();
				geometry_msgs::Pose target_pose;
				target_pose.position.x = current_target.x();
				target_pose.position.y = current_target.y();
				target_pose.position.z = current_target.theta();

				pub_target_pose.publish(target_pose);

				bot_location = navigation_space::DummyNavigator::getBotLocation();

				geometry_msgs::Pose bot_pose;
				bot_pose.position.x = bot_location.x();
				bot_pose.position.y = bot_location.y();
				bot_pose.position.z = bot_location.theta();

				pub_bot_pose.publish(bot_pose);

				break;

			}

			case TrackWaypoint://needs to be at a waypoint already
			{
				heading = message->yaw;
				bool are_waypoints_=navigation_space::TrackWaypointStrategy::readPublishTargetLocation();
				
				if (are_waypoints_==true)
				{	
					if(dist(navigation_space::TrackWaypointStrategy::getBotLocation(),navigation_space::TrackWaypointStrategy::getTargetLocation(heading))<=PROXIMITY)
					{
						are_waypoints_=navigation_space::TrackWaypointStrategy::readPublishTargetLocation();
						if(are_waypoints_==true)
						{
							while(dist(navigation_space::TrackWaypointStrategy::getBotLocation(),navigation_space::TrackWaypointStrategy::getTargetLocation(heading))>=PROXIMITY)
							{
								followWaypoints(navigation_space::TrackWaypointStrategy::getBotLocation(),navigation_space::TrackWaypointStrategy::getTargetLocation(heading));
							}
						}
					}
				}

				break;
			}

			case LaneFollowingOnly:

			case LaserTestOnly:
            case FusionTestOnly:
            case PlannerTestOnly:
            {
            	break;
            }
            case switch_lane_GPS:
            {
            	heading = message->yaw;
            	bool is_previous_waypoint_=false;
            	bool is_next_waypoint_ = navigation_space::TrackWaypointStrategy::readPublishTargetLocation();
            	while (is_next_waypoint_==true && is_previous_waypoint_==false)
            	{
            		while(dist(navigation_space::TrackWaypointStrategy::getBotLocation(),navigation_space::TrackWaypointStrategy::getTargetLocation(heading))>=PROXIMITY)
            		{
            			followLane();
            		}

            		is_previous_waypoint_=true;
            	}

            	while (is_next_waypoint_==true && is_previous_waypoint_==true)
            		{
             			while(dist(navigation_space::TrackWaypointStrategy::getBotLocation(),navigation_space::TrackWaypointStrategy::getTargetLocation(heading))>=PROXIMITY)
            			{
            				followWaypoints(navigation_space::TrackWaypointStrategy::getBotLocation(),navigation_space::TrackWaypointStrategy::getTargetLocation(heading));
            			}

            			is_next_waypoint_ = navigation_space::TrackWaypointStrategy::readPublishTargetLocation();       			
            		}

            	if (is_next_waypoint_==false)
            	{
            		followLane();
            	}
            }

		}
	}



int main(int argc, char* argv[]){

	ros::init(argc,argv,"Navigation");

	ros::NodeHandle nh;
	
	if (argc<2)
	{
		std::cout<<"Usage:Navigator <Number corresponding to strategy>"<<std::endl;
		std::cout<<"FollowNose = 0,\nTrackWaypoint = 1,\nHectorSLAM = 2,\nLaserTestOnly = 3,\nPlannerTestOnly = 4,\nFusionTestOnly = 5,\nIGVCBasic = 6,\nLaneFollowingOnly = 7,\nswitch_lane_GPS = 8,\nDummyNavigator = 9"<<std::endl;
		return 1;
	}

	iterations = 0;
	int strategy = (int)(argv[1][0]-'0'); //Input strategy from terminal

	std::cout<<strategy<<"\n";

	ros::Subscriber sub_imu_sparkFun = nh.subscribe("imuRaw", 10, pose_update);
	ros::Subscriber sub_pose_from_target = nh.subscribe("sensors/pose/",10, navigation_space::TrackWaypointStrategy::setTargetPose);

	pub_target_pose = nh.advertise<geometry_msgs::Pose>("/target_pose",10);
	pub_bot_pose = nh.advertise<geometry_msgs::Pose>("/bot_pose",10);
	
	ros::Rate loop_rate(LOOP_RATE);

	ROS_INFO("Navigation Pub-Sub started \n");

	while(ros::ok()){
		iterations++;
		// ROS_INFO("I was here \n");
		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_INFO("Navigation Exited");

	return 0;
}

