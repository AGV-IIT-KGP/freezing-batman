#include "Navigation/Navigation.h"

navigation::State pose;

ros::Publisher pub_target_pose;
ros::Publisher pub_bot_pose;

int iterations, strategy;

void pose_update(const Navigation::RazorImu::ConstPtr message){
	
	// ROS_INFO("strategy : %d iterations : %d \n", strategy, iterations);

	double heading;
	navigation::State target_location, bot_location;

	switch(strategy){
			case IGVCBasic:
			{
				// Initialize heading with yaw value (Check how)
				// Might need multiple subscribers with synchronization
				// target_location = 
				break;
			}

			case FollowNose:
			{
				heading = message->yaw;
				navigation_space::FollowNoseStrategy::calibrateReferenceHeading(heading, iterations);

				if(iterations < 5)
					break;

				target_location = navigation_space::FollowNoseStrategy::getTargetLocation(heading);
				
				geometry_msgs::Pose target_pose;
				target_pose.position.x = target_location.x();
				target_pose.position.y = target_location.y();
				target_pose.position.z = target_location.theta();

				pub_target_pose.publish(target_pose);

				bot_location = navigation_space::FollowNoseStrategy::getBotLocation();

				geometry_msgs::Pose bot_pose;
				bot_pose.position.x = bot_location.x();
				bot_pose.position.y = bot_location.y();
				bot_pose.position.z = bot_location.theta();

				pub_bot_pose.publish(bot_pose);

				break;
			}

			case TrackWayPoint:
			{
				break;
			}

			case LaserTestOnly:
            case FusionTestOnly:
            case PlannerTestOnly:
            {
            	break;
            }

		}
	}



int main(int argc, char* argv[]){

	ros::init(argc,argv,"Navigation");

	ros::NodeHandle nh;
	
	iterations = 0;
	int strategy = (int)(argv[1][0]-'0'); //Input strategy from terminal

	std::cout<<strategy<<"\n";

	ros::Subscriber sub_imu_sparkFun = nh.subscribe("imuRaw", 10, pose_update);
	
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

