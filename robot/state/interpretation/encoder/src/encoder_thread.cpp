#include "encoder.h"

int main() {
    
    encoder_space::Encoder encoder(ENCODER_COM_PORT, ENCODER_BAUD_RATE);
    encoder_space::EncoderData encoderData;

    ros::NodeHandle n;
    ros::Publisher encoder_pub = n.advertise<geometry_msgs::Pose2D>("encoderData", 1000);
   
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        /* Fetch data from Shaft Encoder and load it in local vars */
        encoderData = encoder.fetchEncoderData();


        geometry_msgs::Pose2D encoder_msg;
        encoder_pub.publish(encoder_msg);

        ros::spinOnce();
        loop_rate.sleep();

        // pthread_mutex_lock(&pose_mutex);

        /* Update the pose_data using the data in local vars */

        // pthread_mutex_unlock(&pose_mutex);

        // usleep(10);
    }

    return 0;
}

