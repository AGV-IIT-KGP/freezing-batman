#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <laneDetector.hpp>

void exitWithHelp() {
    std::cout <<
            "Usage: lane-detector [options]\n"
            "options:\n"
            "-d  : Toggle debug mode, parameter:/lane_detector_test/debug_mode\n"
            "	   0 -- Debug mode OFF\n"
            "	   1 -- Debug mode ON\n"
            "	   2 -- Debug mode ON with obstacle detector thresholding toolBox.\n"
            "	   3 -- Debug mode ON with Canny Hough thresholding toolBox.\n"
            "	   4 -- Debug mode ON with Binary thresholding toolBox.\n"
            "      5 -- Set inverse perspective transform matrix. \n"
            "      6 -- Debug mode ON for inverse perspective transform. \n"
            "      7 -- Inverse perspective transform off, so resizing image. \n"
            "      9 -- Blah Blah \n"
            "-i  : Node Id, parameter: /lane_detector_test/node_id\n"
            "-s  : Subscriber topic name, parameter: /lane_detector_test/subscribe_topic_name\n"
            "-t  : Time functions, parameter:/lane_detector_test/time_Functions\n"
            "-f  : Train file , parameter: /lane_detector_test/train_file\n"
            "      1: Net FPS\n"
            "      2: Individual functions\n"
            "**************new addition to manual****************\n"
            "To change any parameter, write:- [syntax] rosparam set parameter_name value [syntax].\n"
            "for eg.- [command_line]rosparam set /lane_detector/node_id 0[command_line] will set node_id to 0.\n"
            "write [command_line]rosparam list[command_line] for details on parameters and [command_line]rosparam get /[command_line] for present values.\n"
            ;
    exit(1);
}

int main(int argc, char *argv[]) {
    std::string node_name = argv[1];
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    
    // parse options
    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-') {
            exitWithHelp();
        }
        if (argv[i][0] != '-') {
            break;
        }
    }

    LaneDetector lane_detector(node_handle);

    ros::spin();
    return 0;
}
