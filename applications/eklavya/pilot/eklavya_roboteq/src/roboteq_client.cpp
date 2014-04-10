#include "ros/ros.h"
#include "eklavya_roboteq/SetSpeed.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller_client");
    if (argc != 3)
    {
      ROS_INFO("usage: motor_controller_client");
      return 1;
    }
  
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<eklavya_roboteq::SetSpeed>("motor_controller");
    eklavya_roboteq::SetSpeed srv;
    srv.request.left_speed = atoll(argv[1]);
    srv.request.right_speed = atoll(argv[2]);
    if (client.call(srv))
    {
      ROS_INFO("Speed target set to : %ld, %ld", (long int)srv.request.left_speed, (long int)srv.request.right_speed);
      ROS_INFO("Response : %ld", (long int)srv.response.code);
    }
    else
    {
      ROS_ERROR("Failed to call service set speed");
      ROS_INFO("Response : %ld", (long int)srv.response.code);
      return 1;
    }
    
    return 0;
  }
