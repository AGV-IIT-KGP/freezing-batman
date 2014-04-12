#include "my_fusion/fusion.h"


using namespace sensor_msgs;
using namespace message_filters;


void callback(const ImageConstPtr& image, const ImageConstPtr& lidar)
{
 	cv_bridge::CvImagePtr cv_image;
 	cv_bridge::CvImagePtr cv_lidar;
 	
    try {
		
		cv_image = cv_bridge::toCvCopy(image, image_encodings::MONO8);
		cv_lidar = cv_bridge::toCvCopy(lidar, image_encodings::MONO8);

		cv::Mat world_map = cv::Mat::zeros(cv_image->image.size(),CV_8UC1);

		for (int i = 0; i < world_map.rows; i++) {
			for (int j = 0; j < world_map.cols; j++) {
				if (cv_image->image.at<uchar>(i,j) == 255 || cv_lidar->image.at<uchar>(i,j) == 255) {
					world_map.at<uchar>(i,j) = 255;
				} 
				else {
					world_map.at<uchar>(i,j) = 0;
				}
			}
		}

    	cv_bridge::CvImage message;
    	message.encoding = image_encodings::MONO8;
    	message.image = world_map;
    	pub_worldmap.publish(message.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

void singleCallback(const ImageConstPtr& image) {
 	cv_bridge::CvImagePtr cv_image;
 	try {
		
		cv_image = cv_bridge::toCvCopy(image, image_encodings::MONO8);
		
    	cv_bridge::CvImage message;
    	message.encoding = image_encodings::MONO8;
    	message.image = cv_image->image;
    	pub_worldmap.publish(message.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "fusion");

//   ros::NodeHandle nh;

//   image_transport::ImageTransport image_transporter(nh);
//   publisher = image_transporter.advertise("/world_map",10);

//   message_filters::Subscriber<Image> image_sub(nh, "sensors/camera/", 1);
//   message_filters::Subscriber<Image> lidar_sub(nh, "sensors/lidar/", 1);
//   TimeSynchronizer<Image, Image> sync(image_sub, lidar_sub, 10);

//   sync.registerCallback(boost::bind(&callback, _1, _2));
//   ros::spin();
//   return 0;
// }
