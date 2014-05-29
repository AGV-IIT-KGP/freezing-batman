#include <data_fuser/fusion.h>

using namespace sensor_msgs;
using namespace message_filters;

image_transport::Publisher world_map_publisher;

void exit_with_help() {
    std::cout <<
            "Usage: fusion [options]\n"
            "options:\n"
            "-d  : Debug\n"
            "-f  : First Subscriber topic name\n"
            "-s  : Second Subscriber topic name\n"
            "-i  : Node Id\n"
            ;
    exit(1);
}

int main(int argc, char** argv) {
    std::string node_name = std::string("data_fuser");
    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle node_handle;

    int first_subscriber_flag = 1, second_subscriber_flag = 0, debug_flag = 1;
    node_handle.getParam("first_subscriber_flag", first_subscriber_flag);
    node_handle.getParam("second_subscriber_flag", second_subscriber_flag);
    node_handle.getParam("debug_flag", debug_flag);

    std::string first_subscriber_topic_name, second_subscriber_topic_name, node_id, publisher_topic_name;
    first_subscriber_topic_name = std::string("/obstacle_detector/obstacles");
    second_subscriber_topic_name = std::string("/lane_detector/lanes");
    node_id = std::string("0");
    node_handle.getParam("first_subscriber_topic_name", first_subscriber_topic_name);
    node_handle.getParam("second_subscriber_topic_name", second_subscriber_topic_name);
    node_handle.getParam("node_id", node_id);

    for (int i = 1; i < argc; i++) {
        if (argv[i][0] != '-') {
            break;
        }

        /*if (++i>=argc) {
                exit_with_help();
        }*/

        switch (argv[i - 1][1]) {
            case 'd':
                debug_flag = atoi(argv[i]);
                break;
            case 'f':
                first_subscriber_topic_name = std::string(argv[i]);
                first_subscriber_flag = 1;
                break;
            case 's':
                second_subscriber_topic_name = std::string(argv[i]);
                second_subscriber_flag = 1;
                break;
            case 'i':
                node_id = std::string(argv[i]);
                break;
            default:
                fprintf(stderr, "Unknown option: -%c\n", argv[i - 1][1]);
                exit_with_help();
        }
    }

    /*if( !fFlag && !sFlag ) {
            printf("No subscriber mentioned\n");
            exit_with_help();
    }*/

    if (debug_flag) {
        if (first_subscriber_flag) {
            std::cout << "\t First Subscribed topic  :\t" << first_subscriber_topic_name << std::endl;
        }
        if (second_subscriber_flag) {
            std::cout << "\t Second Subscribed topic :\t" << second_subscriber_topic_name << std::endl;
        }
    }

    publisher_topic_name = std::string("/data_fuser/map");
    node_handle.getParam("publisher_topic_name", publisher_topic_name);
    node_name = node_name + node_id;

    image_transport::ImageTransport image_transporter(node_handle);
    world_map_publisher = image_transporter.advertise(publisher_topic_name.c_str(), 10);

    message_filters::Subscriber<Image> first_subscriber;
    message_filters::Subscriber<Image> second_subscriber;
    TimeSynchronizer<Image, Image> *sync;

    image_transport::Subscriber image_subscriber;
            
    if (first_subscriber_flag && second_subscriber_flag) {
        first_subscriber.subscribe(node_handle, first_subscriber_topic_name.c_str(), 1);
        second_subscriber.subscribe(node_handle, second_subscriber_topic_name.c_str(), 1);
        sync = new TimeSynchronizer<Image, Image>(first_subscriber, second_subscriber, 10);
        sync->registerCallback(boost::bind(&callback, _1, _2));
    } else if (first_subscriber_flag) {
        image_subscriber = image_transporter.subscribe(first_subscriber_topic_name, 2, singleCallback);
    } else if (second_subscriber_flag) {
        image_subscriber = image_transporter.subscribe(second_subscriber_topic_name.c_str(), 2, singleCallback);
    }

    ros::spin();
    return 0;
}

