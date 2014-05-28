#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>

int iterations;
double reference_heading;
int map_size = 1000;

ros::Subscriber yaw_subscriber;
ros::Publisher proposed_target_publisher;

geometry_msgs::Pose2D findTarget(double heading) {
    double alpha;
    int x, y, z;
    if (heading * reference_heading > 0) {
        alpha = reference_heading - heading;
    } else {
        if (heading - reference_heading > 180) {
            alpha = 360 + reference_heading - heading;
        } else if (reference_heading - heading > 180) {
            alpha = reference_heading - heading - 360;
        } else {
            alpha = reference_heading - heading;
        }
    }

    alpha *= M_PI / 180.0;

    double map_height = 0.875 * map_size;
    double beta = atan(0.4 * (double) map_size / (double) map_height);
    double gamma = 3.14 - atan(0.4 * (double) map_size / (double) (map_size - map_height));

    if ((-beta <= alpha) && (alpha <= beta)) {
        x = map_height * tan(alpha) + 0.5 * map_size;
        y = map_height + 0.1 * map_size;
    } else if (alpha > beta && alpha < gamma) {
        x = 0.9 * map_size;
        y = 0.4 * map_size / tan(alpha) + 0.1 * map_size;
    } else if (alpha < -beta && alpha > -gamma) {
        x = 0.1 * map_size;
        y = 0.1 * map_size - 0.4 * map_size / tan(alpha);
    } else {
        x = 0.5 * map_size;
        y = 0.1 * map_size;
    }
    z = 0;

    x = x < 100 ? 100 : x;
    y = y < 100 ? 100 : y;

    geometry_msgs::Pose2D target;
    target.x = x;
    target.y = y;
    target.theta = z;

    return target;
}

void publishTarget(const std_msgs::Float64::ConstPtr yaw_msg) {
    geometry_msgs::Pose2D target;
    double heading = yaw_msg->data;

    if (iterations < 5) {
        reference_heading = heading;
        iterations++;
    } else {
        target = findTarget(heading);
        proposed_target_publisher.publish(target);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nose_navigator");
    ros::NodeHandle node_handle;

    iterations = 0;

    std::string proposed_target_topic_name("/nose_navigator/proposed_target");
    std::string yaw_topic_name("/vn_ins/yaw");
    node_handle.getParam("proposed_target_topic_name", proposed_target_topic_name);
    node_handle.getParam("yaw_topic_name", yaw_topic_name);
    
    proposed_target_publisher = node_handle.advertise<geometry_msgs::Pose2D>(proposed_target_topic_name, 10);
    yaw_subscriber = node_handle.subscribe(yaw_topic_name, 10, publishTarget);

    ros::spin();
    return 0;
}