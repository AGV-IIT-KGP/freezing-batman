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
#include <iomanip>

static const int buffer_size = 10;
static const int loop_rate_hz = 10;

sensor_msgs::NavSatFix current_gps;

void setCurrentGPS(sensor_msgs::NavSatFix sensor_gps) {
    current_gps = sensor_gps;
}

std::string doubleToText(const double & d) {
    std::stringstream ss;
    //ss << std::setprecision( std::numeric_limits<double>::digits10+2);
    ss << std::setprecision(std::numeric_limits<int>::max());
    ss << d;
    return ss.str();
}

int main(int argc, char* argv[]) {
    int file_count, i, no_of_waypoints, count;
    std::string c;
    sensor_msgs::NavSatFix term, temp;
    std::string num_file, txtfile_add, file_num_str;
    std::fstream numfile, waypoints_file, edit_numfile;
    std::vector<sensor_msgs::NavSatFix> waypoints_vector;
    sensor_msgs::NavSatFix temp_pair;

    std::cout.unsetf(std::ios::floatfield);
    std::cout.precision(16);

    ros::init(argc, argv, "waypoint_recorder");
    ros::NodeHandle node_handle;

    node_handle.getParam("/waypoint_recorder/data_file", num_file);
    node_handle.getParam("/waypoint_recorder/record_file_folder", txtfile_add);

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
    std::cin.ignore(1, '\n');

    ros::Subscriber sub_current_gps = node_handle.subscribe("/vn_ins/fix", buffer_size, setCurrentGPS);

    ros::Rate loop_rate(loop_rate_hz);
    count = 0;
    while (ros::ok() && count < no_of_waypoints) {
        ros::spinOnce();
        for (i = 0, term.latitude = current_gps.latitude, term.longitude = current_gps.longitude; term.latitude != 0 && term.longitude != 0; ++i) {
            std::cout << "ENTER to record an entry\n'e' to end\nand 'n'(only after 'e') to close file when finished taking waypoints and make a txt." << std::endl;
            std::getline(std::cin, c);
            if (c == "") {
                std::cout << "Reading " << count + 1 << "th waypoint " << i + 1 << "th time." << std::endl;
                ros::spinOnce();
                temp = current_gps;
                std::cout << "latitude:" << temp.latitude << " " << "longitude:" << temp.longitude << std::endl;
                term.latitude = (((term.latitude * i) + temp.latitude) / (i + 1));
                term.longitude = (((term.longitude * i) + temp.longitude) / (i + 1));
                temp_pair.latitude = term.latitude;
                temp_pair.longitude = term.longitude;
                continue;
            } else if (c == "e") {
                std::cout << "\n";
                break;
            } else if (c == "n") {
                std::cout << "\n";
                break;
            }
        }
        if (c == "e") {
            waypoints_vector.push_back(temp_pair);
            count++;
            std::cout << "Read " << count << "th waypoint\n" << "Set to " << temp_pair.latitude << " " << temp_pair.longitude << "\n" << "Move to the next waypoint.\n" << std::endl;
            std::cout << "------------------------------------" << std::endl;
        } else if (c == "n") {
            break;
        }
    }
    edit_numfile.open(num_file.c_str(), std::ios::in | std::ios::out | std::ios::trunc);
    edit_numfile << ++file_count;
    edit_numfile.close();

    std::string record_file = (txtfile_add + "GPS_waypoints" + file_num_str + ".txt");
    std::cout << "Writing to " << record_file << std::endl;
    waypoints_file.open(record_file.c_str(), std::ios::in | std::ios::out | std::ios::trunc);
    waypoints_file << count << std::endl;
    for (std::vector <sensor_msgs::NavSatFix>::iterator it = waypoints_vector.begin(); it != waypoints_vector.end(); it++) {
        std::string textlat = doubleToText(it->latitude);
        std::string textlon = doubleToText(it->latitude);
        waypoints_file << textlat;
        waypoints_file << " ";
        waypoints_file << textlon << std::endl;
    }
    waypoints_file << "#mention this file in the launch of waypoint_selector." << std::endl;
    waypoints_file.close();
}