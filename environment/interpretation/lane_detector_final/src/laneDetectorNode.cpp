#include "laneDetector.hpp"

#include <ros/ros.h>
#include <stdlib.h>
#include <string>

#include <iostream>
#include <vector>

void exit_with_help(){
	std::cout<<
	"Usage: lane-detector [options]\n"
	"options:\n"
	"-d  : Toggle debug mode, parameter:/lane_detector_test/debug_mode\n"
	"	   0 -- Debug mode OFF\n"
	"	   1 -- Debug mode ON\n"
	"	   2 -- Debug mode ON with obstacle detector thresholding toolBox.\n"
	"	   3 -- Debug mode ON with Canny Hough thresholding toolBox.\n"
	"	   4 -- Debug mode ON with Binary thresholding toolBox.\n"
    "      5 -- Set inverse perpective transform matrix. \n"
    "      6 -- Debug mode ON for inverse perspective transform. \n"
    "      7 -- Inverse perspective transform off, so resizing image. \n"
    "      9 -- Blah Blah \n"
	"-i  : Node Id, parameter: /lane_detector_test/node_id\n"
	"-s  : Subscriber topic name, parameter: /lane_detector_test/subscribe_topic_name\n"
	"-t  : Time functions, parameter:/lane_detector_test/time_Functions\n"
	"-f  : Train file , parameter: /lane_detector_test/train_file\n"
	"      1: Net FPS\n"
	"      2: Indivisual functions\n"
	"**************new addition to manual****************\n"
	"To change any parameter, write:- [syntax] rosparam set parameter_name value [syntax].\n"
	"for eg.- [command_line]rosparam set /lane_detector/node_id 0[command_line] will set node_id to 0.\n"
	"write [command_line]rosparam list[command_line] for details on parameters and [command_line]rosparam get /[command_line] for present values.\n"
	;
	exit(1);

}


int main (int argc, char *argv[]){
	
	ros::init(argc,argv,"lane_detector_final");
	
	ros::NodeHandle nh;
	std::string node_id, subscribe_topic_name, train_file;
	int debug_mode  ;
	int time_functions ;
	// the parameters get value from ros parameteres.
	
	nh.getParam("/lane_detector_final/node_id",node_id);
	nh.getParam("/lane_detector_final/subscribe_topic_name",subscribe_topic_name);
	nh.getParam("/lane_detector_final/train_file",train_file);
	nh.getParam("/lane_detector_final/debug_mode",debug_mode);
	nh.getParam("/lane_detector_final/time_functions",time_functions);
	
	
	train_file = std::string("/home/arnatubai/fuerte_workspace/sandbox/freezing-batman/environment/interpretation/lane_detector_final/Samples");
	
	// parse options
	for(int i=1;i<argc;i++) {
		if (argv[i][0]=='-'){// || argv[i].equals("Help") || argv[i].equals("HELP")) {
			exit_with_help();
		}
		if(argv[i][0] != '-') {
			break;
		}
	}
	
	
	std::string node_name, publisher_topic_name;
	node_name = std::string("interpreter_lane_detector_") + node_id;
	publisher_topic_name = std::string("/lane_detector_final/lanes");
	
	
	//ros::NodeHandle nh;

	LaneDetector lane_detector(publisher_topic_name, subscribe_topic_name, time_functions,debug_mode,train_file);
	
	ros::spin();
	
return 0;
}
