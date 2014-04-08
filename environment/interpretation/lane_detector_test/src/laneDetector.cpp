#include "laneDetector.hpp"

LaneDetector::LaneDetector(std::string _pub_topic_name, std::string _sub_topic_name, int _debugMode):it(nh_){
	debug_mode = _debugMode;
	pub_topic_name = _pub_topic_name;
	sub_topic_name = _sub_topic_name;
	setUpCommunication();
}

LaneDetector::~LaneDetector(){
}

void LaneDetector::interpret(){
	/*
	cv::Mat result = Preprocessing(Image);
	if(debug_mode == 1) {
		cv::namedWindow("Preprocessing Output");
		cv::imshow("Preprocessing Output",);
	}
	
	result = GrassRemoval(result);
	if(debug_mode == 1) {
		cv::namedWindow("GrassRemoval Output");
		cv::imshow("GrassRemoval Output",);
	}
	
	result = ObstacleRemoval(result);
	if(debug_mode == 1) {
		cv::namedWindow("ObstacleRemoval Output");
		cv::imshow("ObstacleRemoval Output",);
	}
	
	result = GetLaneBinary(result);
	if(debug_mode == 1) {
		cv::namedWindow("GetLaneBinary Output");
		cv::imshow("GetLaneBinary Output",);
	}
	
	result = SeperateLanes(result);
	if(debug_mode == 1) {
		cv::namedWindow("SeperateLanes Output");
		cv::imshow("SeperateLanes Output",);
	}
	
	result = FixBrokenLanes(result);
	if(debug_mode == 1) {
		cv::namedWindow("FixBrokenLanes Output");
		cv::imshow("FixBrokenLanes Output",);
	}
	
	result = InversePerspectiveTransform(result);
	if(debug_mode == 1) {
		cv::namedWindow("InversePerspectiveTransform Output");
		cv::imshow("InversePerspectiveTransform Output",);
	}
	
	PublishLanes(result);
	*/
}

void LaneDetector::setUpCommunication(){
	pub = it.advertise(pub_topic_name.c_str(), 2);
	sub = it.subscribe(sub_topic_name, 2, &LaneDetector::SubscribeImage, this);
	
	if( debug_mode == 1 ) {
		std::cout << "Communications started with : " << std::endl
				  << "\tSubscriber topic : "<<  sub_topic_name << std::endl
				  << "\tPublisher topic  : "<<  pub_topic_name << std::endl;
	}
}


void LaneDetector::SubscribeImage(const sensor_msgs::ImageConstPtr& msg) {
    try {
        Image = bridge.imgMsgToCv(msg, "bgr8");
        cv::waitKey(WAIT_TIME);
    }
    catch (sensor_msgs::CvBridgeException& e) {
        ROS_ERROR("ERROR IN CONVERTING IMAGE!!!");
    }
    
    if (debug_mode) {
		cv::namedWindow("Original Image");
        cv::imshow("Original Image", Image);
        cv::waitKey(WAIT_TIME);
    }
    
    interpret();
}

void LaneDetector::PublishLanes(cv::Mat &image){
	
	cvi.encoding = sensor_msgs::image_encodings::BGR8;
	cvi.image = image;
	pub.publish(cvi.toImageMsg());
}
