#include "ros/ros.h"
#include "eklavya_roboteq/GetSpeed.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_speed_client");
    if (argc != 1) {
        ROS_INFO("usage: motor_speed_client");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<eklavya_roboteq::GetSpeed > ("motor_speed");
    eklavya_roboteq::GetSpeed srv;
    if (client.call(srv)) {
        ROS_INFO("Speed response : %ld, %ld", (long int) srv.response.left_speed, (long int) srv.response.right_speed);
    } else {
        ROS_ERROR("Failed to call service get speed");
        return 1;
    }

    return 0;
}
