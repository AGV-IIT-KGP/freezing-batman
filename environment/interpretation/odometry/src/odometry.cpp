#include <eklavya_odometry/odometry.h>

namespace odometry_space {

    OdometryFactory::OdometryFactory() {
        sequence_id = 0;

        position_x = 0;
        position_y = 0;
        yaw = 0;

        velocity_x = 0.01;
        yaw_rate = 0.001;

        for (int i = 0; i < 36; i++) {
            if (i / 6 == i % 6) {
                pose_covariance_matrix[i] = 1;
                twist_covariance_matrix[i] = 1;
            } else {
                pose_covariance_matrix[i] = 0;
                twist_covariance_matrix[i] = 0;
            }
        }
        wheel_separation = 0.45;
        scaling_factor = (wheel_radius_meter / 60.0) * 2.0 * PI;
    }

    void OdometryFactory::updateOdometryData(const eklavya_encoder::Encoder_Data::ConstPtr& msg) {
        velocity_x = (msg->left_count + msg->right_count) * (scaling_factor) / 2;
        yaw_rate = (msg->right_count - msg->left_count) * (scaling_factor) / (wheel_separation);
    }

    nav_msgs::Odometry OdometryFactory::getOdometryData() {
        nav_msgs::Odometry odometry_message;

        if (sequence_id == 0) {
            last_time = ros::Time::now();
        }

        current_time = ros::Time::now();
        duration = current_time - last_time;

        position_x += velocity_x * sin(yaw) * duration.toSec();
        position_y += velocity_x * cos(yaw) * duration.toSec();
        std::cout<<"position)_y "<<position_y<<" velocity_x "<<velocity_x<<" duration "<<duration.toSec()<<" cos(yaw) "<<cos(yaw)<<std::endl;
        yaw += yaw_rate * duration.toSec();

        quaternion = tf::createQuaternionMsgFromYaw(yaw);

        //tf update	
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = current_time;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = position_x;
        transform_stamped.transform.translation.y = position_y;
        transform_stamped.transform.translation.z = 0;
        transform_stamped.transform.rotation = quaternion;

        //tf broadcast
        odom_broadcaster.sendTransform(transform_stamped);

        //Odometry message update
        //Header
        odometry_message.header.seq = sequence_id++;
        odometry_message.header.stamp = current_time;
        odometry_message.header.frame_id = "odom";

        //Child Frame
        odometry_message.child_frame_id = "base_link";

        //Twist
        odometry_message.twist.twist.linear.x = velocity_x;
        odometry_message.twist.twist.linear.y = 0; //Fixed
        odometry_message.twist.twist.linear.z = 0; //Fixed

        odometry_message.twist.twist.angular.x = 0; //Fixed
        odometry_message.twist.twist.angular.y = 0; //Fixed
        odometry_message.twist.twist.angular.z = yaw_rate;

        for (int i = 0; i < 36; i++) {
            odometry_message.twist.covariance[i] = twist_covariance_matrix[i];
        }

        //Pose
        odometry_message.pose.pose.position.x = position_x;
        odometry_message.pose.pose.position.y = position_y;
        odometry_message.pose.pose.position.z = 0; //Fixed

        odometry_message.pose.pose.orientation = quaternion;

        for (int i = 0; i < 36; i++) {
            odometry_message.pose.covariance[i] = pose_covariance_matrix[i];
        }

        last_time = current_time;

        return odometry_message;
    }

}
