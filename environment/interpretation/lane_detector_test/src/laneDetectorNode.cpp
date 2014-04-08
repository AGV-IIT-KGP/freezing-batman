#include "laneDetector.hpp"

#include <ros/ros.h>
#include <stdlib.h>
#include <string>

void exit_with_help(){
	std::cout<<
	"Usage: lane-detector [options]\n"
	"options:\n"
	"-d  : Toggle debug mode\n"
	"	   0 -- Debug mode OFF\n"
	"	   1 -- Debug mode ON\n"
	"-i  : Node Id\n"
	"-s  : Subscriber topic name\n"
	;
	exit(1);

}


int main (int argc, char *argv[]){
	std::string node_id="0", subscribe_topic_name="/camera/image";
	int debug_mode = 0 ;
	// parse options
	for(int i=1;i<argc;i++)
	{
		if(argv[i][0] != '-') {
			break;
		}
		
		if (++i>=argc) {
			exit_with_help();
		}
		switch(argv[i-1][1])
		{
			case 'd':
				debug_mode = atoi(argv[i]);
				break;
			case 'i':
				node_id = std::string(argv[i]);
				break;
			case 's':
				subscribe_topic_name = std::string(argv[i]);
				break;
			default:
				fprintf(stderr,"Unknown option: -%c\n", argv[i-1][1]);
				exit_with_help();
		}
	}
	std::string node_name, publisher_topic_name;
	node_name = std::string("interpreter_lane_detector_") + node_id;
	publisher_topic_name = std::string("/interpreter/lane_detector/") + node_id;
	
	ros::init(argc, argv, node_name.c_str());
	//ros::NodeHandle nh;

	LaneDetector lane_detector(publisher_topic_name, subscribe_topic_name, debug_mode);
	
	ros::spin();
return 0;
}
