#include <iostream>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <local_planner/Seed.h>

#define LOOP_RATE 10

enum pilot_modes {
    low_granularity = 0,
    high_granularity
};

int pilot_mode;
ros::Publisher command_publisher;

void publishCommand(const local_planner::Seed temp) {
    geometry_msgs::Twist cmdvel;
    local_planner::Seed seed = temp;

    int left_vel = 0;
    int right_vel = 0;
    float left_velocity = seed.leftVelocity;
    float right_velocity = seed.rightVelocity;

    if ((left_velocity == 0) && (right_velocity == 0)) {
        ROS_INFO("Braking");
        left_vel = left_velocity;
        right_vel = right_velocity;
    } else if ((left_velocity >= 0) && (right_velocity >= 0)) {
        switch (pilot_mode) {
            case low_granularity:
            {
                if (left_velocity == right_velocity) {
                    double vavg = 80;
                    left_vel = right_vel = vavg;
                    printf("straight seed\n");
                } else if (seed.velocityRatio == 1.258574 || seed.velocityRatio == 0.794550) {
                    double vavg = 50;
                    double aggression = 1;
                    seed.velocityRatio = seed.velocityRatio < 1 ? seed.velocityRatio / aggression : seed.velocityRatio * aggression;
                    left_vel = (int) 2 * vavg * seed.velocityRatio / (1 + seed.velocityRatio);
                    right_vel = (int) (2 * vavg - left_vel);
                    printf("soft seed\n");
                } else if (seed.velocityRatio == 1.352941 || seed.velocityRatio == 0.739130) {
                    double vavg = 20;
                    double aggression = 1.5;
                    seed.velocityRatio = seed.velocityRatio < 1 ? seed.velocityRatio / aggression : seed.velocityRatio * aggression;
                    left_vel = (int) 2 * vavg * seed.velocityRatio / (1 + seed.velocityRatio);
                    right_vel = (int) (2 * vavg - left_vel);
                    printf("hard seed\n");
                }
                break;
            }
            case high_granularity:
            {
                if (left_velocity == right_velocity) {
                    ROS_INFO("Forward");
                } else if (left_velocity < right_velocity) {
                    ROS_INFO("Left Turn");
                } else {
                    ROS_INFO("Right Turn");
                }
                left_vel = left_velocity;
                right_vel = right_velocity;
                break;
            }
        }
    } else if ((left_velocity < 0) && (right_velocity < 0)) {
        ROS_INFO("Reversing");
        left_vel = left_velocity;
        right_vel = right_velocity;
    } else {
        ROS_INFO("Zero Radius Turn");
        left_vel = left_velocity;
        right_vel = right_velocity;
    }

    left_vel = left_vel > 80 ? 80 : left_vel;
    right_vel = right_vel > 80 ? 80 : right_vel;
    left_vel = left_vel < -80 ? -80 : left_vel;
    right_vel = right_vel < -80 ? -80 : right_vel;

    double scale = 100;
    double w = 0.55000000;
    cmdvel.linear.x = (left_vel + right_vel) / (2 * scale);
    cmdvel.linear.y = 0;
    cmdvel.linear.z = 0;
    cmdvel.angular.x = 0;
    cmdvel.angular.y = 0;
    cmdvel.angular.z = (left_vel - right_vel) / (w * scale);

    command_publisher.publish(cmdvel);
    ROS_INFO("[eklavya3_pilot] Command : (%d, %d)", left_vel, right_vel);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "eklavya3_pilot");
    ros::NodeHandle node_handle; // nodeHandle
    node_handle.getParam("eklavya3_pilot/pilot_mode", pilot_mode);
    
    command_publisher = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 10); //Publisher for Control
    ros::Subscriber seed_subscriber = node_handle.subscribe("local_planner/seed", 10, publishCommand); //Subscriber for Path

    ros::spin();
    return 0;
}
