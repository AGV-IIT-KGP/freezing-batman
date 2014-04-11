#include "laneDetector.hpp"

LaneDetector::LaneDetector(std::string _pub_topic_name, std::string _sub_topic_name, int _timeFunctions, int _debugMode):it(nh_){
	debug_mode = _debugMode;
	timeFunctions = _timeFunctions;
	pub_topic_name = _pub_topic_name;
	sub_topic_name = _sub_topic_name;
	
	// Grass Removal
	kernel_size = 8;
	svm = new SVM();
	svm->init(kernel_size*kernel_size*3);
	svm->loadModel("Samples.model");
	
	setUpCommunication();
}

LaneDetector::~LaneDetector(){
}

void LaneDetector::interpret(){
	
	totalTimeElapsed = 0;
	
	cv::Mat result = Image;
	
	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
	result = Preprocessing(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		std::cout << "Preprocessing FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("Preprocessing Output");
		cv::imshow("Preprocessing Output",result);
	}
	*/
	
	if( timeFunctions ){
	 	gettimeofday (&tvalBefore, NULL);
	}	
	result = GrassRemoval(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		std::cout << "GrassRemoval FPS : "<< 1./timeElapsed << std::endl;
	//	std::cout << "Time Elapsed        : "<< timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("GrassRemoval Output");
		cv::imshow("GrassRemoval Output",result);
	}
	
	
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}	
	result = ObstacleRemoval(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		std::cout << "ObstacleRemoval FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("ObstacleRemoval Output");
		cv::imshow("ObstacleRemoval Output",result);
	}
	
	
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}	
	result = GetLaneBinary(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		std::cout << "GetLaneBinary FPS : "<< 1./timeElapsed << std::endl;
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
		totalTimeElapsed += timeElapsed;
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
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		std::cout << "FixBrokenLanes FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("FixBrokenLanes Output");
		cv::imshow("FixBrokenLanes Output",result);
	}
	*/

	/*
	if( timeFunctions ){
		gettimeofday (&tvalBefore, NULL);
	}
	result = InversePerspectiveTransform(result);
	if( timeFunctions ){
		gettimeofday (&tvalAfter, NULL);
		timeElapsed = tvalAfter.tv_sec+(tvalAfter.tv_usec/1000000.0) - (tvalBefore.tv_sec+(tvalBefore.tv_usec/1000000.0));
		totalTimeElapsed += timeElapsed;
		std::cout << "InversePerspectiveTransform FPS : "<< 1./timeElapsed << std::endl;
	}
	if(debug_mode) {
		cv::namedWindow("InversePerspectiveTransform Output");
		cv::imshow("InversePerspectiveTransform Output", result);
	}
	*/
	
	if( timeFunctions ){
		std::cout << "Total FPS : "<< 1./totalTimeElapsed << std::endl;;
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
    message.encoding = sensor_msgs::image_encodings::BGR8;
    
    cv::Mat BGR_image(image.rows,image.cols,CV_8UC1);
	cv::cvtColor(image,BGR_image,CV_GRAY2BGR);
    
    message.image = BGR_image;
	pub.publish(message.toImageMsg());
}
