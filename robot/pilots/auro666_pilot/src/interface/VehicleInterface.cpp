#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <auro666_pilot/SetSteering.h>
#include <auro666_pilot/SetBrake.h>
#include <auro666_pilot/SetThrottle.h>
#include <auro666_pilot/GetState.h>

ros::ServiceClient steering_client;
ros::ServiceClient brake_client;
ros::ServiceClient throttle_client;
ros::ServiceClient state_client;

void setSteering(const std_msgs::Int64::ConstPtr& msg) {
    auro666_pilot::SetSteering srv;
    srv.request.steering = msg->data;

    if (steering_client.call(srv)) {
        ROS_INFO("Steering target set to : %ld", (long int) srv.request.steering);
        ROS_INFO("Response : %ld", (long int) srv.response.code);
    } else {
        ROS_ERROR("Failed to call service roboteq/steering");
        ROS_INFO("Response : %ld", (long int) srv.response.code);
    }
}

void setBrake(const std_msgs::Int64::ConstPtr& msg) {
    auro666_pilot::SetBrake srv;
    srv.request.brake = msg->data;

    if (brake_client.call(srv)) {
        ROS_INFO("Brake target set to : %ld", (long int) srv.request.brake);
        ROS_INFO("Response : %ld", (long int) srv.response.code);
    } else {
        ROS_ERROR("Failed to call service roboteq/brake");
        ROS_INFO("Response : %ld", (long int) srv.response.code);
    }
}

void setThrottle(const std_msgs::Int64::ConstPtr& msg) {
    ROS_INFO("[VehicleInterface] throttle = %ld", msg->data);
    
    auro666_pilot::SetThrottle srv;
    srv.request.throttle = msg->data;

    if (throttle_client.call(srv)) {
        ROS_INFO("Throttle target set to : %ld", (long int) srv.request.throttle);
        ROS_INFO("Response : %ld", (long int) srv.response.code);
    } else {
        ROS_ERROR("Failed to call service atmega/throttle");
        ROS_INFO("Response : %ld", (long int) srv.response.code);
    }
}

std_msgs::Int64 getEncoderCounts() {
    std_msgs::Int64 encoder_counts;
    auro666_pilot::GetState srv;

    if (state_client.call(srv)) {
        ROS_INFO("Encoder value obtained: %ld", (long int) srv.response.encoder_counts);
        encoder_counts.data = srv.response.encoder_counts;
    } else {
        ROS_ERROR("Failed to call service atmega/state");
    }

    return encoder_counts;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_interface");
    ros::NodeHandle node_handle;

    steering_client = node_handle.serviceClient<auro666_pilot::SetSteering>("roboteq/steering");
    brake_client = node_handle.serviceClient<auro666_pilot::SetBrake>("roboteq/brake");
    throttle_client = node_handle.serviceClient<auro666_pilot::SetThrottle>("atmega/throttle");
    state_client = node_handle.serviceClient<auro666_pilot::GetState>("atmega/state");
    
    ros::Subscriber steering_subscriber = node_handle.subscribe("interface/controls/steering", 2, setSteering);
    ros::Subscriber brake_subscriber = node_handle.subscribe("interface/controls/brake", 2, setBrake);
    ros::Subscriber throttle_subscriber = node_handle.subscribe("interface/controls/throttle", 2, setThrottle);
    ros::Publisher encoder_publisher = node_handle.advertise<std_msgs::Int64>("interface/state/encoder_counts", 100);

    ros::Rate loop_rate(2);
    while (ros::ok()) {
//        ROS_INFO("[VehicleInterface] Heart Beat");
        
        std_msgs::Int64 encoder_counts;
        //encoder_counts = getEncoderCounts();
        encoder_publisher.publish(encoder_counts);

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

