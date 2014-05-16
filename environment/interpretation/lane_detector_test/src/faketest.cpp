

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



int main(int argc, char **argv)
{
	cv::Mat array[3];
	cv::Mat frame;
	array[0]=cv::imread(argv[1]);
	array[1]=cv::imread(argv[2]);
	array[2]=cv::imread(argv[3]);
	
	int r;


	ros::init(argc, argv, "Lane_detection_tester");
	ros::NodeHandle node_handle;
	int frame_id=0;
	std::string  node_name = "camera";
    	std::string topic_name = "/camera/image";
    	int message_queue_size = 10;

	 

    	image_transport::ImageTransport image_transporter(node_handle);
    	image_transport::Publisher publisher = image_transporter.advertise(topic_name.c_str(), message_queue_size);

    	while(ros::ok())
    	{
    		r=std::rand()%3;
    		frame=array[r];
    		cv_bridge::CvImage message;
    		message.header.seq = frame_id;
    		message.header.frame_id = frame_id;
    		message.header.stamp = ros::Time::now();
    		message.encoding = sensor_msgs::image_encodings::BGR8;
    		message.image = frame;

    		publisher.publish(message.toImageMsg());




    	}



    	return 0;

}


   
