#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dummy_navigator");
    ros::NodeHandle node_handle;

    ros::Publisher target_publisher = node_handle.advertise<geometry_msgs::Pose2D>("dummy_navigator/target", 10);
    
    geometry_msgs::Pose2D target;
    target.x = 10.;
    target.y = 0.;
    target.theta = M_PI / 2;
    
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        node_handle.getParam("dummy_navigator/target_x", target.x);
        node_handle.getParam("dummy_navigator/target_y", target.y);
        node_handle.getParam("dummy_navigator/target_theta", target.theta);
        target_publisher.publish(target);
        loop_rate.sleep();
    }

    return 0;
}