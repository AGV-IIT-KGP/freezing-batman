#include "ObstacleDetector.hpp"	

void ObstacleDetector::publishData(){
	cv_bridge::CvImage out_msg;
	out_msg.image    = img;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
	pub.publish(out_msg.toImageMsg());
}

void ObstacleDetector::interpret() {
	if (DEBUG){
        cvNamedWindow("Control Box", 1);
    }
    int s=5;
    cvCreateTrackbar("Kernel 1", "Control Box", &s, 20);

    if (DEBUG) {
		cv::namedWindow("Raw Scan", 0);
		cv::imshow("Raw Scan", img);
		cv::waitKey(WAIT_TIME);
    }

	int dilation_size = EXPAND_OBS;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       cv::Point( dilation_size, dilation_size ) );
    cv::dilate(img, img, element);

    if (DEBUG) {
        cv::namedWindow("Dilate Filter", 1);
        cv::imshow("Dilate Filter", img);
        cv::waitKey(WAIT_TIME);
    }
publishData();
}

void ObstacleDetector::scanCallback(const sensor_msgs::LaserScan& scan) {
	size_t size = scan.ranges.size();
    float angle = scan.angle_min;
    float maxRangeForContainer = scan.range_max - 0.1f;
	
	img = img-img;
	
    for (size_t i = 0; i < size; ++i) {
        float dist = scan.ranges[i];
        if ((dist > scan.range_min) && (dist < maxRangeForContainer)) {
            double x1 = -1 * sin(angle) * dist;
            double y1 = cos(angle) * dist;
            int x = (int) ((x1 * 100) + CENTERX);
            int y = (int) ((y1 * 100) + CENTERY + LIDAR_Y_SHIFT);


            if (x >= 0 && y >= 0 && (int) x < MAP_MAX && (int) y < MAP_MAX) {
                int x2 = (x);
                int y2 = (MAP_MAX - y - 30 - 1);

                img.at<uchar>(y2,x2)=255;
            }
        }
        angle += scan.angle_increment;
    }
    interpret();
}

ObstacleDetector::ObstacleDetector(int argc, char *argv[],ros::NodeHandle &node_handle):nh(node_handle) {
    if (argc>1){
	 topic_name = std::string("interpreter/obstacleMap/") + std::string(argv[1]);
	}
	else{
	 topic_name = std::string("interpreter/obstacleMap/0");
	}
	if (argc>2){
	 sub_topic_name = std::string(argv[2]);
	}
	else{
	 sub_topic_name = std::string("/scan");
	}

    it = new image_transport::ImageTransport(nh);
    sub = nh.subscribe(sub_topic_name.c_str(), 2, &ObstacleDetector::scanCallback,this);
    img = cv::Mat(MAP_MAX,MAP_MAX,CV_8UC1,cvScalarAll(0));
    pub = it->advertise(topic_name.c_str(), 10);

}

ObstacleDetector::~ObstacleDetector() {
}

int main(int argc, char** argv) {
	std::string node_name;
	if (argc>1){
	 node_name = std::string("interpreter_obstacleDetector_") + std::string(argv[1]);
	}
	else{
	 node_name = std::string("interpreter_obstacleDetector_0");
	}

    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle nh;
    ObstacleDetector obstacle_detector(argc, argv, nh);

    ros::Rate loop_rate(LOOP_RATE);
    ROS_INFO("Obstacle Detector Thread Started...");
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Obstacle code exiting");
    return 0;
}
