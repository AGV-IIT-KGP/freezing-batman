#include "eklavya3_Pilot/controller.hpp"

ros::Publisher pub_control;

int main(int argc,char* argv[]) {

    ros::init(argc, argv, "controller");

    ros::NodeHandle nh; // nodeHandle

    pub_control = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); //Publisher for Control

    ros::Subscriber sub_path = nh.subscribe("/path",10, sendCommand); //Subscriber for Path

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok()) {
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Controller Exited");
    return 0;
}
