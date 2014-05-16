#include "my_fusion/fusion.h"

using namespace sensor_msgs;
using namespace message_filters;

image_transport::Publisher pub_worldmap;

void exit_with_help(){
	std::cout<<
	"Usage: fusion [options]\n"
	"options:\n"
	"-d  : Debug\n"
	"-f  : First Subscriber topic name\n"
	"-s  : Second Subscriber topic name\n"
	"-i  : Node Id\n"
	;
	exit(1);

}

int main(int argc, char** argv)
{
	int fFlag = 0, sFlag = 0, debugFlag = 0;
	
	std::string first_sub_topic_name, second_sub_topic_name, node_id, node_name, publisher_topic_name;
	
	first_sub_topic_name = std::string("/obstacle_detector/obstacles");
	second_sub_topic_name = std::string("/interpreter/lane_detector/0");
	node_id = std::string("0");
	node_name = std::string("interpreter_fusion_");
	for(int i=1;i<argc;i++) {
		if(argv[i][0] != '-') {
			break;
		}
		
		if (++i>=argc) {
			exit_with_help();
		}
		
		switch(argv[i-1][1]) {
			case 'd':
				debugFlag = atoi(argv[i]);
				break;
			case 'f':
				first_sub_topic_name = std::string(argv[i]);
				fFlag = 1;
				break;
			case 's':
				second_sub_topic_name = std::string(argv[i]);
				sFlag = 1;
				break;
			case 'i':
				node_id = std::string(argv[i]);
				break;
			default:
				fprintf(stderr, "Unknown option: -%c\n", argv[i-1][1]);
				exit_with_help();
		}
	}
	
	if( !fFlag && !sFlag ) {
		printf("No subscriber mentioned\n");
		exit_with_help();
	}
	
	if( debugFlag ) {
		if( fFlag ) {
			std::cout<<"\t First Subscribed topic  :\t"<<first_sub_topic_name<<std::endl;
		}
		if( sFlag ) {
			std::cout<<"\t Second Subscribed topic :\t"<<first_sub_topic_name<<std::endl;
		}
	}
	
	publisher_topic_name = std::string("interpreter/fusion/world_map/") + node_id;
	
	node_name = node_name + node_id;
	
	ros::init(argc, argv, node_name.c_str());
	ros::NodeHandle nh;
	
	image_transport::ImageTransport image_transporter(nh);
	pub_worldmap = image_transporter.advertise(publisher_topic_name.c_str(),10);
	
	message_filters::Subscriber<Image> first_sub;
	message_filters::Subscriber<Image> second_sub;
	TimeSynchronizer<Image, Image> *sync;
	
	image_transport::Subscriber sub;
	
	if( fFlag && sFlag ) {
		first_sub.subscribe(nh, first_sub_topic_name.c_str() , 1);
		second_sub.subscribe(nh, second_sub_topic_name.c_str(), 1);
		sync       = new TimeSynchronizer<Image, Image>(first_sub, second_sub, 10);
		sync->registerCallback(boost::bind(&callback, _1, _2));
	}
	else if( fFlag ) {
		 sub = image_transporter.subscribe(first_sub_topic_name.c_str(), 2, singleCallback);
	}
	else if( sFlag ) {
		 sub = image_transporter.subscribe(second_sub_topic_name.c_str(), 2, singleCallback);
	}
	
	ros::spin();
return 0;
}
