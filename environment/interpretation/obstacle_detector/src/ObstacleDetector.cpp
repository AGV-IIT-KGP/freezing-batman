#include "../include/ObstacleDetector.hpp"
#include <climits>
#include <cassert>

void exit_with_help(){
	std::cout<<
	"Usage: lane-detector [options]\n"
	"options:\n"
	"-d  : Non-zero for debug\n"
	"-s  : Subscriber topic name\n"
    "-p  : Publisher topic name\n"
    "-l  : Maximum distance we need \n"
    "-m  : Mininum distance we need \n"
	;
	exit(1);

}
void ObstacleDetector::publishData(){
	cv_bridge::CvImage out_msg;
	out_msg.encoding = sensor_msgs::image_encodings::MONO8;
	out_msg.image    = img;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
	pub.publish(out_msg.toImageMsg());
}

void ObstacleDetector::interpret() {
	if (debug){
        cvNamedWindow("Control Box", 1);
    }
    int s=5;
    
    if (debug) {
		cvCreateTrackbar("Kernel 1", "Control Box", &s, 20);
		cv::namedWindow("Raw Scan", 0);
		cv::imshow("Raw Scan", img);
		cv::waitKey(WAIT_TIME);
    }

	int dilation_size = EXPAND_OBS;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       cv::Point( dilation_size, dilation_size ) );
    cv::dilate(img, img, element);

    if (debug) {
        cv::namedWindow("Dilate Filter", 1);
        cv::imshow("Dilate Filter", img);
        cv::waitKey(WAIT_TIME);
    }
publishData();
}

void ObstacleDetector::scanCallback(const sensor_msgs::LaserScan& scan) {
    ROS_INFO("Scan callback called");
	size_t size = scan.ranges.size();
    float angle = scan.angle_min;
    float maxRangeForContainer = scan.range_max - 0.1f;

    img = img-img; // Assign zero to all pixels
    for (size_t i = 0; i < size; ++i) {
        float dist = scan.ranges[i];
        if ((dist > scan.range_min) && (dist < maxRangeForContainer)) {
            double x1 = -1 * sin(angle) * dist;
            double y1 = cos(angle) * dist;
            int x = (int) ((x1 * 100) + CENTERX);
            int y = (int) ((y1 * 100) + CENTERY + LIDAR_Y_SHIFT);


            if (x >= 0 && y >= min_dist && (int) x < MAP_MAX && (int) y < max_dist) {
                int x2 = (x);
                int y2 = (MAP_MAX - y - 30 - 1);
                if(!(y2 >= 0 && y2 < MAP_MAX)){
                    continue;
                }

                img.at<uchar>(y2,x2)=255;
            }
        }
        angle += scan.angle_increment;
    }
    interpret();
}

ObstacleDetector::ObstacleDetector(int argc, char *argv[], ros::NodeHandle &node_handle):nh(node_handle) {

	topic_name = std::string("interpreter/obstacleMap/0");
	sub_topic_name = std::string("/scan");
    min_dist = 0;
	max_dist = 400;
	debug = 0;
	for(int i=1;i<argc;i++)
	{
		if(argv[i][0] != '-') {
			break;
		}

		if (++i>=argc) {
		}
		switch(argv[i-1][1])
		{
			case 'd':
				debug = atoi(argv[i]);
				break;
			case 's':
				sub_topic_name = std::string(argv[i]);
				break;
            case 'p':
                topic_name = std::string(argv[i]);
                break;
            case 'l':
                max_dist = atoi(argv[i]);
                break;
            case 'm':
                min_dist = atoi(argv[i]);
                break;
			default:
				fprintf(stderr, "Unknown option: -%c\n", argv[i-1][1]);
				exit_with_help();
		}
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
	// if (argc>1){
	//  node_name = std::string("interpreter_obstacleDetector_") + std::string(argv[1]);
	// }
	// else{
	//  node_name = std::string("interpreter_obstacleDetector_0");
	// }

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
