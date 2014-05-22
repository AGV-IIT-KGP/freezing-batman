#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <string>
#include <cmath>
#include <iostream>
#include <ios>
#include <iosfwd>
#include <fstream>

static const int buffer_size = 10;
static const int loop_rate_hz = 10;

sensor_msgs::NavSatFix current_gps;

void setCurrentGPS(sensor_msgs::NavSatFix sensor_gps) {
    current_gps = sensor_gps;
}

int main(int argc, char* argv[]) {
    int file_count, i, no_of_waypoints, count;
    char c;
    float term_lat, term_long;
    std::string num_file, txtfile_add, file_num_str;
    std::fstream numfile, waypoints_file, edit_numfile;
    std::vector<std::pair<float, float> > waypoints_vector;
    std::pair<float, float> temp_pair;

    ros::init(argc, argv, "waypoint_recorder");
    ros::NodeHandle node_handle;

    node_handle.getParam("/waypoint_recorder/data_file", num_file);
    node_handle.getParam("/waypoint_recorder/record_file_folder", txtfile_add);
//    std::cout << num_file << std::endl;
//    std::cout << txtfile_add << std::endl;

    numfile.open(num_file.c_str());

    if (numfile.is_open()) {
        numfile>>file_count;
        std::stringstream ss;
        ss << file_count;
        file_num_str = ss.str();
        numfile.close();
    } else {
        file_count = 0;
        numfile.close();
    }
    std::cout << "Enter the no of waypoints:";
    std::cin>>no_of_waypoints;
    std::cout << "ENTER to record an entry\n'e' to end\nand 'n'(only after 'e') to close file when finished taking waypoints and make a txt." << std::endl;

    ros::Subscriber sub_current_gps = node_handle.subscribe("/vn_ins/fix", buffer_size, setCurrentGPS);

    ros::Rate loop_rate(loop_rate_hz);
    count = 0;
    while (ros::ok() && count < no_of_waypoints) {
        for (i = 0, term_lat = 0, term_long = 0;; i++) {
            c = getchar();
            if (c == '\n') {
                std::cout<<"Reading "<<count+1<<"th waypoint"<<i+1<<"th time."<<std::endl;
                ros::spinOnce();
                term_lat += ((term_lat * i) + current_gps.latitude) / (i + 1);
                term_long += ((term_long * i) + current_gps.longitude) / (i + 1);
                temp_pair.first = term_lat;
                temp_pair.second = term_long;
                continue;
            } else if (c == 'e') {
                break;
            } else if (c == 'n') {
                break;
            }
        }
        if (c == 'e') {
            waypoints_vector.push_back(temp_pair);
            count++;
            std::cout<<"Read "<<count<<"th waypoint"<<std::endl;
            std::cout<<"------------------------------------"<<std::endl;
        } else if (c == 'n') {
            break;
        }
    }
    edit_numfile.open(num_file.c_str(), std::ios::in | std::ios::out | std::ios::trunc);
    edit_numfile << ++file_count;
    edit_numfile.close();

    std::string record_file = (txtfile_add + "GPS_waypoints" + file_num_str + ".txt");
    std::cout<<"Writing to "<<record_file<<std::endl;
    waypoints_file.open(record_file.c_str(),std::ios::in | std::ios::out | std::ios::trunc);
    waypoints_file << count << std::endl;
    for (std::vector < std::pair<float, float> >::iterator it = waypoints_vector.begin(); it != waypoints_vector.end(); it++) {
        waypoints_file << it->first;
        waypoints_file << " ";
        waypoints_file << it->second << std::endl;
    }
    waypoints_file << "#mention this file in the launch of waypoint_selector." << std::endl;
    waypoints_file.close();
}