/* 
 * File:   AtmegaServer.hpp
 * Author: samuel
 *
 * Created on 28 January, 2014, 6:52 PM
 */

#ifndef ATMEGASERVER_HPP
#define	ATMEGASERVER_HPP

#include <string>
#include <ros/ros.h>
#include <auro666_pilot/SetThrottle.h>
#include <auro666_pilot/GetState.h>
#include <interface/atmega/AtmegaDevice.hpp>

class AtmegaServer {
public:
    AtmegaServer();
    AtmegaServer(ros::NodeHandle node_handle);
    AtmegaServer(const AtmegaServer& orig);
    virtual ~AtmegaServer();
    
    void shutdown();
    
private:
    AtmegaDevice throttle_controller;
    AtmegaDevice encoder_reader;
    
    ros::ServiceServer throttle_service;
    ros::ServiceServer state_service;

    bool getState(auro666_pilot::GetState::Request &req, auro666_pilot::GetState::Response &res);
    void initialize();
    bool setThrottle(auro666_pilot::SetThrottle::Request &req, auro666_pilot::SetThrottle::Response &res);
};

#endif	/* ATMEGASERVER_HPP */

