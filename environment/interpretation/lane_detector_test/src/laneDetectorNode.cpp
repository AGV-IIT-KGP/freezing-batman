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
	"	   2 -- Debug mode ON with obstacle detector thresholding toolBox.\n"
	"	   3 -- Debug mode ON with Canny Hough thresholding toolBox.\n"
	"	   4 -- Debug mode ON with Binary thresholding toolBox.\n"
    "      5 -- Set inverse perpective transform matrix. \n"
	"-i  : Node Id\n"
	"-s  : Subscriber topic name\n"
	"-t  : Time the Functions\n"
	;
	exit(1);

}


int main (int argc, char *argv[]){
	std::string node_id="0", subscribe_topic_name="/camera/image";
	int debug_mode = 0 ;
	int time_functions=0;
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
			case 't':
				time_functions = atoi(argv[i]);
				break;
			default:
				fprintf(stderr, "Unknown option: -%c\n", argv[i-1][1]);
				exit_with_help();
		}
	}
	std::string node_name, publisher_topic_name;
	node_name = std::string("interpreter_lane_detector_") + node_id;
	publisher_topic_name = std::string("/interpreter/lane_detector/") + node_id;
	
	ros::init(argc, argv, node_name.c_str());
	//ros::NodeHandle nh;

	LaneDetector lane_detector(publisher_topic_name, subscribe_topic_name, time_functions,debug_mode);
	
	ros::spin();
return 0;
}
