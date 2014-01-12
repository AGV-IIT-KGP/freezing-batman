#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <auro666_pilot/Controls.h>
#include <auro666_pilot/State.h>
#include <control/SteeringController.hpp>
#include <control/CruiseController.hpp>

SteeringController steering_controller;
CruiseController cruise_controller;

void updatePose(const geometry_msgs::Pose::ConstPtr& pose_ptr) {
    steering_controller.SetPose(pose_ptr);
    cruise_controller.SetPose(pose_ptr);
}

void updateState(const auro666_pilot::State::ConstPtr& state_ptr) {
    steering_controller.SetState(state_ptr);
}

void updatePath(const nav_msgs::Path::ConstPtr& path_ptr) {
    steering_controller.SetPath(path_ptr);
    cruise_controller.SetPath(path_ptr);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle node_handle;

    // Note:
    // Can't use the approximate time policy to have a single callback function
    // The control algo should run irrespective of the incoming messages, as long as the path isn't complete
    ros::Subscriber pose_subscriber = node_handle.subscribe("localization/pose", 2, updatePose);
    ros::Subscriber state_subscriber = node_handle.subscribe("vehicle_server/state", 2, updateState);
    ros::Subscriber path_subscriber = node_handle.subscribe("local_planner/path", 2, updatePath);

    ros::Publisher controls_publisher = node_handle.advertise<auro666_pilot::Controls>("controller/controls", 20);
    ros::Publisher cte_publisher = node_handle.advertise<std_msgs::Float64>("controller/cross_track_error", 20);

    auro666_pilot::Controls controls;
    std_msgs::Float64 cte_msg;

    ros::Rate loop_rate(20);
    while (ros::ok()) {
        controls.rear_wheel_speed = cruise_controller.getCruiseControl();
        controls.steer_angle = steering_controller.getSteeringControl();
        cte_msg.data = steering_controller.GetCte();

        controls_publisher.publish(controls);
        cte_publisher.publish(cte_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    controls.rear_wheel_speed = 0;
    controls.steer_angle = 0;
    controls_publisher.publish(controls);

    return 0;
}
