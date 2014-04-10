#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "eklavya_roboteq/SetSpeed.h"

ros::NodeHandle *n;
ros::ServiceClient *client;

void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    eklavya_roboteq::SetSpeed srv;

    int sum = msg->linear.x * 100, diff = (msg->angular.z * 55) / 2;
    //srv.request.left_speed = (sum + diff) * 10;
    //srv.request.right_speed = (sum - diff) * 10;
    srv.request.right_speed = (sum + diff) * 10;
    srv.request.left_speed = (sum - diff) * 10;
    std::cout << "  !m  " << srv.request.left_speed << " " << srv.request.right_speed << std::endl;
//    std::cout << "linear: " << msg->linear.x << " angular: " << msg->angular.z << std::endl;
    
    if (client->call(srv)) {
        ROS_INFO("Speed target set to : %ld, %ld", (long int) srv.request.left_speed, (long int) srv.request.right_speed);
        ROS_INFO("Response : %ld", (long int) srv.response.code);
    } else {
        ROS_ERROR("Failed to call service set speed");
        ROS_INFO("Response : %ld", (long int) srv.response.code);
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_subscriber_motor_controller_client");

    ros::NodeHandle n1;
    n = &n1;

    ros::ServiceClient serviceClient = n->serviceClient<eklavya_roboteq::SetSpeed > ("motor_controller");

    client = &serviceClient;

    ros::Subscriber sub = n->subscribe("cmd_vel", 1, commandVelocityCallback);

    ros::spin();

    return 0;
}
