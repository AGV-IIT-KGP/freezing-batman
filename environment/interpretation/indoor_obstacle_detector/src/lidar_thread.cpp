#include "LidarData.h"



char **laser_scan;



int main(int argc,char** argv)  {
    std::string node_name("indoor_obstacle_detector");
    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle lidar_node;
	LidarData lidardata;
	
	image_transport::ImageTransport imagetransport(lidar_node);
    lidardata.obstacle_publisher = imagetransport.advertise(std::string("/") + node_name + std::string("/obstacles"), 10);
    ros::Subscriber scan_subscriber = lidar_node.subscribe("/sensors/hokuyo_nodes/0", 2, &LidarData::update_map, &lidardata);
    //ros::Subscriber scan_subscriber = lidar_node.subscribe("/scan", 2, &LidarData::update_map, &lidardata);
	cv::Mat showImage1(400,400,CV_8UC1,cvScalarAll(0));
	cv::Mat showImage2(400,400,CV_8UC1,cvScalarAll(0));
	cv::Mat showImage3(400,400,CV_8UC1,cvScalarAll(0));
    ros::Rate loop_rate(10);

   

#ifdef FPS_TEST
    ROS_INFO("[LIDAR] Conducting an FPS Test");
    int iterations = 0;
    time_t start = time(0);
#endif

    while (ros::ok()) {
#ifdef FPS_TEST
        if (iterations > 100) {
            time_t finish = time(0);
            double fps = (iterations + 0.0) / (finish - start);
            ROS_INFO("[LIDAR] FPS: %lf", fps);
            break;
        }

        iterations++;
#endif

        ros::spinOnce();
        loop_rate.sleep();
    }

    return NULL;
}

