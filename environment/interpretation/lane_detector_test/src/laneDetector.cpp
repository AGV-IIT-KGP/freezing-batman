#include "laneDetector.hpp"
#include "ros/package.h"
#include <stdio.h>
#include <iostream>

int do_preprocessing = 1;
int do_grass_removal = 1;
int do_obstacle_removal = 1;
int do_get_lane_binary = 1;
int do_inverse_perspective_transform = 1;

void loadVariables(){

    int status=1;
    std::string path = ros::package::getPath("lane_detector_test")+"/select.txt";
    FILE *readFile;
    readFile = fopen(path.c_str(),"r");
    status = status && fscanf(readFile, "preprocessing = %d", &do_preprocessing);
    status = status && fscanf(readFile, "grass_removal = %d", &do_grass_removal);
    status = status && fscanf(readFile, "obstacle_removal = %d", &do_obstacle_removal);
    status = status && fscanf(readFile, "get_lane_binary = %d", &do_get_lane_binary);
    status = status && fscanf(readFile, "inverse_perspective_transform = %d", &do_inverse_perspective_transform);
    if(!status){
        printf("in reading bot variable file ");
        printf(path.c_str());
    }
        
}

LaneDetector::LaneDetector(std::string _pub_topic_name, std::string _sub_topic_name, int _timeFunctions, int _debugMode):it(nh_){
	debug_mode = _debugMode;
	timeFunctions = _timeFunctions;
	pub_topic_name = _pub_topic_name;
	sub_topic_name = _sub_topic_name;

	// Grass Removal
	kernel_size = 8;
	svm = new SVM();
	svm->init(kernel_size*kernel_size*3);
    std::string model_path = ros::package::getPath("lane_detector_test")+"/Samples.model";
	svm->loadModel(model_path.c_str());
	setUpCommunication();
}

LaneDetector::~LaneDetector(){
}

void LaneDetector::interpret(){
	
	totalTimeElapsed = 0;
	cv::Mat result = Image;


	
	if( timeFunctions == 2 ){
		gettimeofday (&tvalBefore, NULL);
	}
    if(do_preprocessing){
        result = Preprocessing(result);
    }
	if( timeFunctions == 2 ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		std::cout << "Preprocessing FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("Preprocessing Output");
		cv::imshow("Preprocessing Output",result);
	}
	

	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
    if(do_grass_removal){
        result = GrassRemoval(result);
    }
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		if( timeFunctions == 2 ) {
			std::cout << "GrassRemoval FPS : "<< 1./timeElapsed << std::endl;
		}
	}
	if(debug_mode) {
		cv::namedWindow("GrassRemoval Output");
		cv::imshow("GrassRemoval Output",result);
	}

	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
    if(do_obstacle_removal){
        result = ObstacleRemoval(result);
    }
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		if( timeFunctions == 2 ) {
			std::cout << "ObstacleRemoval FPS : "<< 1./timeElapsed << std::endl;
		}
	}
	if(debug_mode) {
		cv::namedWindow("ObstacleRemoval Output");
		cv::imshow("ObstacleRemoval Output",result);
	}

	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
    if(do_get_lane_binary){
        result = GetLaneBinary(result);
    }
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		if( timeFunctions == 2 ) {
			std::cout << "GetLaneBinary FPS : "<< 1./timeElapsed << std::endl;
		}
	}
	if(debug_mode) {
		cv::namedWindow("GetLaneBinary Output");
		cv::imshow("GetLaneBinary Output",result);
	}
	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
	result = SeperateLanes(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		std::cout << "SeperateLanes FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("SeperateLanes Output");
		cv::imshow("SeperateLanes Output",result);
	}
	*/
	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
	result = FixBrokenLanes(result);
	if( timeFunctions == 2 ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		if( timeFunctions == 2 ) {
			std::cout << "FixBrokenLanes FPS : "<< 1./timeElapsed << std::endl;
		}
	}
	if(debug_mode) {
		cv::namedWindow("FixBrokenLanes Output");
		cv::imshow("FixBrokenLanes Output",result);
	}
	*/

	
	if( timeFunctions == 2 ){
		gettimeofday (&tvalBefore, NULL);
	}
    if(do_inverse_perspective_transform){
        result = InversePerspectiveTransform(result);
    }
	if( timeFunctions == 2 ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		if( timeFunctions == 2 ) {
			std::cout << "InversePerspectiveTransform FPS : "<< 1./timeElapsed << std::endl;
		}
	}
	if(debug_mode) {
		cv::namedWindow("InversePerspectiveTransform Output");
		cv::imshow("InversePerspectiveTransform Output", result);
	}

	
	if( timeFunctions ){
		if( timeFunctions == 2 ) {
			std::cout << "Total FPS : "<< 1./totalTimeElapsed << std::endl;
		}
		else {
			ROS_INFO("Total FPS : %lf", 1./totalTimeElapsed );
		}
	}
	

	PublishLanes(result);
}

void LaneDetector::setUpCommunication(){

	pub = it.advertise(pub_topic_name.c_str(), 2);
	sub = it.subscribe(sub_topic_name, 2, &LaneDetector::SubscribeImage, this);
	if( debug_mode ) {
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
	
	cv_bridge::CvImage message;
    message.encoding = sensor_msgs::image_encodings::MONO8;
    if( debug_mode == 7 ) {
		message.image = cv::Mat::zeros(MAP_MAX,MAP_MAX,CV_8UC1);
		cv::resize(image, message.image, message.image.size());
	}
	else {
		message.image = image;
	}
	pub.publish(message.toImageMsg());

}
