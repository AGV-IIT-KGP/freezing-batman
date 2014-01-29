/* 
 * File:   AtmegaServer.cpp
 * Author: samuel
 * 
 * Created on 28 January, 2014, 6:52 PM
 */

#include <interface/atmega/AtmegaServer.hpp>

AtmegaServer::AtmegaServer() {
}

AtmegaServer::AtmegaServer(ros::NodeHandle node_handle) {
    initialize();

    throttle_controller.connect();
    //encoder_reader.connect();

    // TODO: Initiate communication with Atmega

    throttle_service = node_handle.advertiseService("atmega/throttle", &AtmegaServer::setThrottle, this);
    state_service = node_handle.advertiseService("atmega/state", &AtmegaServer::getState, this);
}

AtmegaServer::AtmegaServer(const AtmegaServer& orig) {
}

AtmegaServer::~AtmegaServer() {
}

bool AtmegaServer::getState(auro666_pilot::GetState::Request& req,
                            auro666_pilot::GetState::Response& res) {
    // Fill in res.encoder_counts

    int data;
    encoder_reader.recieve(data, 6);
    res.encoder_counts = data;
    
    return true;
}

bool AtmegaServer::setThrottle(auro666_pilot::SetThrottle::Request& req,
                               auro666_pilot::SetThrottle::Response& res) {
    ROS_INFO("[AtmegaServer] throttle = %ld", req.throttle);
    
    throttle_controller.send((int) req.throttle, 3);
    
    return true;
}

void AtmegaServer::initialize() {
    throttle_controller.SetComm_port(std::string("/dev/ttyUSB0"));
    throttle_controller.SetBaud_rate(9600);

    encoder_reader.SetComm_port(std::string("/dev/ttyUSB1"));
    encoder_reader.SetBaud_rate(9600);
}

void AtmegaServer::shutdown() {
    // TODO: Disconnect from Atmega

    throttle_controller.disconnect();
    encoder_reader.disconnect();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "atmega_server");
    ros::NodeHandle node_handle;
    AtmegaServer atmega_server(node_handle);

    ros::spin();

    atmega_server.shutdown();
}
